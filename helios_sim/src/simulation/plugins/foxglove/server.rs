// helios_sim/src/simulation/plugins/foxglove/server.rs
//
// Async tokio WebSocket server — runs on a dedicated OS thread.
//
// Binds on both 0.0.0.0 (IPv4) and [::1] (IPv6) so that "localhost" resolves
// correctly on macOS, where getaddrinfo returns ::1 before 127.0.0.1.

use crate::simulation::plugins::foxglove::protocol::{
    encode_message_frame, AdvertiseMessage, BridgeMessage, ChannelAdvertisement, ServerControl,
    ServerInfo,
};
use futures_util::{SinkExt, StreamExt};
use std::collections::HashMap;
use std::sync::{Arc, RwLock};
use tokio::net::TcpListener;
use tokio::sync::{broadcast, mpsc};
use tokio_tungstenite::accept_hdr_async;
use tokio_tungstenite::tungstenite::handshake::server::{ErrorResponse, Request, Response};
use tokio_tungstenite::tungstenite::Message;

/// Entry point for the background tokio server.
///
/// `data_rx`  — receives `BridgeMessage` from the Bevy main thread.
/// `ctrl_tx`  — sends `ServerControl` back to the Bevy main thread.
pub async fn run_server(
    port: u16,
    mut data_rx: mpsc::UnboundedReceiver<BridgeMessage>,
    ctrl_tx: mpsc::UnboundedSender<ServerControl>,
    channel_snapshot: Arc<RwLock<Vec<ChannelAdvertisement>>>,
) {
    // Broadcast channel: relay task → all connected client tasks.
    let (broadcast_tx, _) = broadcast::channel::<BridgeMessage>(2048);
    let broadcast_tx_relay = broadcast_tx.clone();

    // Relay task: bridge the unbounded mpsc → broadcast.
    tokio::spawn(async move {
        while let Some(msg) = data_rx.recv().await {
            let _ = broadcast_tx_relay.send(msg);
        }
    });

    // Bind on both IPv4 and IPv6 so that "localhost" resolves on macOS/Linux.
    // On macOS, getaddrinfo("localhost") returns ::1 before 127.0.0.1, so
    // binding only 0.0.0.0 leaves IPv6 connections unreachable.
    let v4_addr = format!("0.0.0.0:{port}");
    let v6_addr = format!("[::1]:{port}");

    let listener_v4 = TcpListener::bind(&v4_addr).await;
    let listener_v6 = TcpListener::bind(&v6_addr).await;

    match (&listener_v4, &listener_v6) {
        (Err(e4), Err(e6)) => {
            bevy::log::error!("[foxglove] Could not bind on any address — v4: {e4}, v6: {e6}");
            return;
        }
        _ => {
            bevy::log::info!("[foxglove] WebSocket server listening on ws://localhost:{port}");
        }
    }

    // Spawn an independent accept loop for each bound address.
    if let Ok(l) = listener_v4 {
        let btx = broadcast_tx.clone();
        let ctx = ctrl_tx.clone();
        let snap = channel_snapshot.clone();
        tokio::spawn(accept_loop(l, btx, ctx, snap));
    }
    if let Ok(l) = listener_v6 {
        let btx = broadcast_tx.clone();
        let ctx = ctrl_tx.clone();
        let snap = channel_snapshot.clone();
        tokio::spawn(accept_loop(l, btx, ctx, snap));
    }

    // Keep this task alive indefinitely so the runtime doesn't exit.
    std::future::pending::<()>().await;
}

// ---------------------------------------------------------------------------
// Accept loop (one per bound address)
// ---------------------------------------------------------------------------

async fn accept_loop(
    listener: TcpListener,
    broadcast_tx: broadcast::Sender<BridgeMessage>,
    ctrl_tx: mpsc::UnboundedSender<ServerControl>,
    channel_snapshot: Arc<RwLock<Vec<ChannelAdvertisement>>>,
) {
    loop {
        match listener.accept().await {
            Ok((stream, peer_addr)) => {
                bevy::log::info!("[foxglove] Client connecting from {peer_addr}");
                // Snapshot the current channel list so handle_client can send
                // an initial advertise before entering the select! loop.
                let initial_channels = channel_snapshot
                    .read()
                    .map(|s| s.clone())
                    .unwrap_or_default();
                let broadcast_rx = broadcast_tx.subscribe();
                let ctrl_tx_clone = ctrl_tx.clone();
                tokio::spawn(handle_client(
                    stream,
                    peer_addr,
                    broadcast_rx,
                    ctrl_tx_clone,
                    initial_channels,
                ));
            }
            Err(e) => {
                bevy::log::warn!("[foxglove] TCP accept error: {e}");
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Per-client task
// ---------------------------------------------------------------------------

async fn handle_client(
    stream: tokio::net::TcpStream,
    addr: std::net::SocketAddr,
    mut broadcast_rx: broadcast::Receiver<BridgeMessage>,
    ctrl_tx: mpsc::UnboundedSender<ServerControl>,
    initial_channels: Vec<ChannelAdvertisement>,
) {
    // Foxglove Studio requires the server to echo back the foxglove.websocket.v1
    // subprotocol in the HTTP 101 response, otherwise it rejects the connection.
    let ws = match accept_hdr_async(stream, foxglove_handshake).await {
        Ok(ws) => ws,
        Err(e) => {
            bevy::log::warn!("[foxglove] WS handshake failed for {addr}: {e}");
            return;
        }
    };
    bevy::log::info!("[foxglove] Client {addr} connected");

    let (mut ws_tx, mut ws_rx) = ws.split();

    // Send serverInfo immediately.
    let info = ServerInfo {
        op: "serverInfo",
        name: "Helios Simulation".to_string(),
        capabilities: vec!["clientPublish".to_string()],
        supported_encodings: vec!["json".to_string()],
        session_id: std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_millis()
            .to_string(),
    };
    if let Ok(json) = serde_json::to_string(&info) {
        let _ = ws_tx.send(Message::Text(json.into())).await;
    }

    // Advertise all channels known at connection time.
    // Without this, a client that connects after the sim has started would
    // never see topics registered before it connected.
    if !initial_channels.is_empty() {
        let adv = AdvertiseMessage {
            op: "advertise",
            channels: initial_channels,
        };
        if let Ok(json) = serde_json::to_string(&adv) {
            let _ = ws_tx.send(Message::Text(json.into())).await;
        }
    }

    // Per-client subscription tracking: sub_id ↔ channel_id.
    let mut sub_to_channel: HashMap<u32, u32> = HashMap::new();
    let mut channel_to_sub: HashMap<u32, u32> = HashMap::new();

    loop {
        tokio::select! {
            ws_msg = ws_rx.next() => {
                match ws_msg {
                    None => break,
                    Some(Err(e)) => {
                        bevy::log::debug!("[foxglove] Client {addr} WS error: {e}");
                        break;
                    }
                    Some(Ok(Message::Text(text))) => {
                        handle_client_message(
                            &text,
                            &mut sub_to_channel,
                            &mut channel_to_sub,
                            &ctrl_tx,
                        );
                    }
                    Some(Ok(Message::Close(_))) => break,
                    Some(Ok(_)) => {}
                }
            }

            bcast = broadcast_rx.recv() => {
                match bcast {
                    Err(broadcast::error::RecvError::Closed) => break,
                    Err(broadcast::error::RecvError::Lagged(n)) => {
                        bevy::log::warn!("[foxglove] Client {addr} lagged {n} messages");
                    }
                    Ok(BridgeMessage::Data { channel_id, timestamp_ns, payload }) => {
                        if let Some(&sub_id) = channel_to_sub.get(&channel_id) {
                            let frame = encode_message_frame(sub_id, timestamp_ns, &payload);
                            if ws_tx.send(Message::Binary(frame.into())).await.is_err() {
                                break;
                            }
                        }
                    }
                    Ok(BridgeMessage::Advertise(channels)) => {
                        let adv = AdvertiseMessage { op: "advertise", channels };
                        if let Ok(json) = serde_json::to_string(&adv) {
                            if ws_tx.send(Message::Text(json.into())).await.is_err() {
                                break;
                            }
                        }
                    }
                }
            }
        }
    }

    for channel_id in channel_to_sub.keys().copied() {
        let _ = ctrl_tx.send(ServerControl::Unsubscribe { channel_id });
    }
    bevy::log::info!("[foxglove] Client {addr} disconnected");
}

// ---------------------------------------------------------------------------
// WebSocket handshake callback — echoes the foxglove.websocket.v1 subprotocol.
// ---------------------------------------------------------------------------

fn foxglove_handshake(req: &Request, response: Response) -> Result<Response, ErrorResponse> {
    const FOXGLOVE_PROTO: &str = "foxglove.websocket.v1";
    let mut response = response;

    // Chrome Private Network Access (PNA): when the Foxglove web app (served
    // from https://studio.foxglove.dev) connects to localhost, Chrome injects
    // `Access-Control-Request-Private-Network: true` into the upgrade request
    // and requires the server to echo `Access-Control-Allow-Private-Network: true`
    // in the 101 response. Without this header Chrome blocks the connection
    // before it reaches our accept loop — the server log shows nothing.
    if req
        .headers()
        .get("Access-Control-Request-Private-Network")
        .and_then(|v| v.to_str().ok())
        .unwrap_or("")
        == "true"
    {
        response.headers_mut().insert(
            "Access-Control-Allow-Private-Network",
            "true".parse().expect("valid header value"),
        );
    }

    // Foxglove protocol subprotocol negotiation.
    if req
        .headers()
        .get("Sec-WebSocket-Protocol")
        .and_then(|v| v.to_str().ok())
        .unwrap_or("")
        .contains(FOXGLOVE_PROTO)
    {
        response.headers_mut().insert(
            "Sec-WebSocket-Protocol",
            FOXGLOVE_PROTO.parse().expect("valid header value"),
        );
    }

    Ok(response)
}

// ---------------------------------------------------------------------------
// Parse subscribe / unsubscribe messages from the Foxglove client.
// ---------------------------------------------------------------------------

fn handle_client_message(
    text: &str,
    sub_to_channel: &mut HashMap<u32, u32>,
    channel_to_sub: &mut HashMap<u32, u32>,
    ctrl_tx: &mpsc::UnboundedSender<ServerControl>,
) {
    let v: serde_json::Value = match serde_json::from_str(text) {
        Ok(v) => v,
        Err(_) => return,
    };

    match v.get("op").and_then(|o| o.as_str()) {
        Some("subscribe") => {
            let subs = match v.get("subscriptions").and_then(|s| s.as_array()) {
                Some(a) => a,
                None => return,
            };
            for sub in subs {
                let sub_id = sub.get("id").and_then(|i| i.as_u64()).unwrap_or(0) as u32;
                let channel_id = sub.get("channelId").and_then(|c| c.as_u64()).unwrap_or(0) as u32;
                sub_to_channel.insert(sub_id, channel_id);
                channel_to_sub.insert(channel_id, sub_id);
                let _ = ctrl_tx.send(ServerControl::Subscribe { channel_id });
            }
        }
        Some("unsubscribe") => {
            let ids = match v.get("subscriptionIds").and_then(|s| s.as_array()) {
                Some(a) => a,
                None => return,
            };
            for id in ids {
                let sub_id = id.as_u64().unwrap_or(0) as u32;
                if let Some(channel_id) = sub_to_channel.remove(&sub_id) {
                    channel_to_sub.remove(&channel_id);
                    let _ = ctrl_tx.send(ServerControl::Unsubscribe { channel_id });
                }
            }
        }
        _ => {}
    }
}

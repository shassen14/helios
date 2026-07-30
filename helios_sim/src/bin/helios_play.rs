//! `helios_play` — the interactive, windowed launcher for a Helios scenario.
//!
//! The real-time twin of `helios_test_sim`: both add the same `HeliosHost` body
//! and differ only in what they wrap around it. The test bin runs headless, owns
//! an exit-code verdict, and layers the assertion runner on top; this bin adds
//! nothing — it opens a window, runs the scenario, and lets a human watch.
//!
//! `--headless` still routes here, selecting the windowless host, so this one
//! binary is both the interactive player and a plain scenario launcher. There is
//! no verdict to report, so the `AppExit` that `run()` returns is discarded —
//! unlike the test bin, where the exit code *is* the pass/fail contract.

use helios_sim::cli::Cli;
use helios_sim::prelude::*;
use helios_sim::viz::interaction::ActionRegistryPlugin;
use helios_sim::viz::VizPlugin;

use clap::Parser;
use helios_sim::viz::interaction::camera::CameraPlugin;
use helios_sim::viz::interaction::selection::SelectionPlugin;

fn main() {
    let cli = Cli::parse();

    let presentation = if cli.headless {
        Presentation::Headless
    } else {
        Presentation::Windowed
    };

    let time_policy = TimePolicy::from_cli(cli.speed, presentation);

    let mut app = App::new();

    app.add_plugins(HeliosHost::new(cli, presentation, time_policy));

    if presentation == Presentation::Windowed {
        app.add_plugins(ActionRegistryPlugin);
        app.add_plugins(VizPlugin);
        app.add_plugins(CameraPlugin);
        app.add_plugins(SelectionPlugin);
    }

    app.run();
}

use crate::{
    port::{ChannelKey, PortBus, SensorChannel},
    prelude::{AgentRuntime, TickContext},
};
use helios_core::{
    estimation::EstimatorInputs,
    prelude::{AngularVelocity3D, LinearAcceleration3D, SensorReading},
};
use nalgebra::DVector;

pub(crate) trait EstimatorInputBuilder: Send + Sync {
    fn assemble(
        &self,
        bus: &PortBus,
        runtime: &dyn AgentRuntime,
        tick: &TickContext,
    ) -> Option<EstimatorInputs>;

    fn required_channels(&self) -> &[ChannelKey];

    fn optional_channels(&self) -> &[ChannelKey];
}

/// Assembles a 6-element IMU control vector `[ax, ay, az, wx, wy, wz]` from
/// the most recent linear acceleration and angular velocity readings on the bus.
/// Returns `None` if either channel is empty (cold-start or sensor dropout).
pub(crate) struct IntegratedImuInputBuilder {
    accel_channel: ChannelKey,
    gyro_channel: ChannelKey,
    required: Vec<ChannelKey>,
}

impl Default for IntegratedImuInputBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl IntegratedImuInputBuilder {
    pub(crate) fn new() -> Self {
        let accel_channel: ChannelKey =
            SensorChannel::of::<Vec<SensorReading<LinearAcceleration3D>>>().into();
        let gyro_channel: ChannelKey =
            SensorChannel::of::<Vec<SensorReading<AngularVelocity3D>>>().into();

        Self {
            accel_channel: accel_channel.clone(),
            gyro_channel: gyro_channel.clone(),
            required: vec![accel_channel, gyro_channel],
        }
    }
}

impl EstimatorInputBuilder for IntegratedImuInputBuilder {
    fn assemble(
        &self,
        bus: &PortBus,
        _runtime: &dyn AgentRuntime,
        _tick: &TickContext,
    ) -> Option<EstimatorInputs> {
        let accel_stamped =
            bus.read::<Vec<SensorReading<LinearAcceleration3D>>>(self.accel_channel.clone())?;
        let gyro_stamped =
            bus.read::<Vec<SensorReading<AngularVelocity3D>>>(self.gyro_channel.clone())?;

        let accel = accel_stamped.value.last()?;
        let gyro = gyro_stamped.value.last()?;

        let control = DVector::from_row_slice(&[
            accel.data.value.x,
            accel.data.value.y,
            accel.data.value.z,
            gyro.data.value.x,
            gyro.data.value.y,
            gyro.data.value.z,
        ]);

        Some(EstimatorInputs { control })
    }

    fn required_channels(&self) -> &[ChannelKey] {
        &self.required
    }

    fn optional_channels(&self) -> &[ChannelKey] {
        &[]
    }
}

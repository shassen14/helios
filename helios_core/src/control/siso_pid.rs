// helios_core/src/control/siso_pid.rs
//
// Generic single-input single-output PID primitive.
// No trait — composable by value into higher-level controllers.

/// A single-input single-output PID controller with anti-windup clamping.
pub struct SisoPid {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    /// Maximum absolute value of the integral accumulator. 0.0 = unclamped.
    pub integral_clamp: f64,
    integral: f64,
    prev_error: f64,
}

impl SisoPid {
    pub fn new(kp: f64, ki: f64, kd: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral_clamp: 0.0,
            integral: 0.0,
            prev_error: 0.0,
        }
    }

    pub fn with_integral_clamp(mut self, clamp: f64) -> Self {
        self.integral_clamp = clamp;
        self
    }

    /// Compute output given error and elapsed time `dt` (seconds).
    pub fn update(&mut self, error: f64, dt: f64) -> f64 {
        self.integral += error * dt;
        if self.integral_clamp > 0.0 {
            self.integral = self
                .integral
                .clamp(-self.integral_clamp, self.integral_clamp);
        }
        let derivative = if dt > 0.0 {
            (error - self.prev_error) / dt
        } else {
            0.0
        };
        self.prev_error = error;
        self.kp * error + self.ki * self.integral + self.kd * derivative
    }

    /// Reset integrator and derivative state.
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
    }
}

//! Closed-loop scenario tests, driving controllers against a simple
//! physical model rather than checking their math in isolation.

use utilities::pid::{PController, PidController};
use utilities::universal_constants::KINDA_SMALL_NUMBER;

/// First-order lag plant, `time_constant * dx/dt = gain * u - x`,
/// integrated with explicit Euler. Used to closed-loop test the
/// controllers against a simple, well-understood dynamic response.
struct FirstOrderPlant {
    state: f64,
    gain: f64,
    time_constant: f64,
}

impl FirstOrderPlant {
    fn new(gain: f64, time_constant: f64) -> Self {
        Self {
            state: 0.0,
            gain,
            time_constant,
        }
    }

    fn step(&mut self, input: f64, dt: f64) -> f64 {
        self.state += dt * (self.gain * input - self.state) / self.time_constant;
        self.state
    }
}

#[test]
fn p_controller_closed_loop_leaves_steady_state_error_on_first_order_plant() {
    let setpoint = 1.0;
    let dt = 0.01;
    let plant_gain = 1.0;
    let proportional_gain = 4.0;

    let mut plant = FirstOrderPlant::new(plant_gain, 1.0);
    let controller = PController::new(proportional_gain);

    let mut measurement = 0.0;
    for _ in 0..5000 {
        let error = setpoint - measurement;
        let command = controller.update(error);
        measurement = plant.step(command, dt);
    }

    // Steady state of a first-order plant under pure P control:
    // x = setpoint * (plant_gain * kp) / (1 + plant_gain * kp).
    let loop_gain = plant_gain * proportional_gain;
    let expected_measurement = setpoint * loop_gain / (1.0 + loop_gain);
    assert!((measurement - expected_measurement).abs() < KINDA_SMALL_NUMBER);

    // P alone cannot cancel the steady-state error on this plant.
    assert!((setpoint - measurement).abs() > 0.1);
}

#[test]
fn pid_controller_closed_loop_eliminates_steady_state_error_on_first_order_plant() {
    let setpoint = 1.0;
    let dt = 0.01;

    let mut plant = FirstOrderPlant::new(1.0, 1.0);
    let mut controller = PidController::new(4.0, 2.0, 0.5);

    let mut measurement = 0.0;
    for _ in 0..5000 {
        let error = setpoint - measurement;
        let command = controller.update(error, dt);
        measurement = plant.step(command, dt);
    }

    // Integral action drives the steady-state error to (near) zero,
    // unlike a pure P controller on the same plant.
    assert!((setpoint - measurement).abs() < KINDA_SMALL_NUMBER);
}

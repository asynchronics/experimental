use utilities::pid::PidController;
use utilities::plot::plot_series;

/// First-order lag plant, `time_constant * dx/dt = gain * u - x`,
/// integrated with explicit Euler.
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

fn main() {
    let setpoint = 1.0;
    let dt = 0.01;
    let steps = 1000;

    let mut plant = FirstOrderPlant::new(1.0, 1.0);
    let mut controller = PidController::new(4.0, 2.0, 0.5);

    let mut measurement = Vec::with_capacity(steps);
    let mut reference = Vec::with_capacity(steps);

    let mut y = 0.0;
    for i in 0..steps {
        let t = i as f64 * dt;
        let error = setpoint - y;
        let command = controller.update(error, dt);
        y = plant.step(command, dt);

        measurement.push([t, y]);
        reference.push([t, setpoint]);
    }

    plot_series(&[("measurement", &measurement), ("setpoint", &reference)]);
}

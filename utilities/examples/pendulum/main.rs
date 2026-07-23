// Running example starts a simulation server.
// Simulation is controlled by a python client so it will start after client.py or client_QT.py is also run.
// To run the example, first run the server with `cargo run --example pendulum`, then run the client with `python3 client.py` or `python3 client_QT.py` in a separate terminal.
// After that the simulation will start and all visualizations will be displayed.
// The program also sends position over UDP to localhost:8080, which can be received using udp_client.py, that can be run with `python3 udp_client.py` in third terminal.
// To setup python environment use following commands:
// `cd /utilities/examples/pendulum`
// `python3 -m venv venv`
// `source venv/bin/activate`
// `pip3 install -r requirements.txt`

use std::error::Error;
use std::net::UdpSocket;
use std::time::Duration;

use serde::{Deserialize, Serialize};

use nexosim::model::{Context, Model, schedulable};
use nexosim::ports::{
    EventSinkReader, EventSlotReader, EventSource, Output, SinkState, UniRequestor,
    event_queue_endpoint, event_slot,
};
use nexosim::server;
use nexosim::simulation::{Mailbox, SimInit, SimulationError};
use nexosim::time::MonotonicTime;

use raylib::prelude::*;
use std::sync::mpsc;
use utilities::angles::{Degrees, Radians};
use utilities::pid::PidController;

use std::f64::consts::PI;

/// Pendulum model
#[derive(Serialize, Deserialize)]
pub struct Pendulum {
    /// Position [rad] -- output port.
    pub position: Output<f64>,
    /// velocity [rad/s] -- output port.
    pub velocity: Output<f64>,
    /// acceleration [rad/s^2] -- output port.
    pub acceleration: Output<f64>,
    /// Energy [J] -- output port.
    pub energy: Output<f64>,
    /// Gravitational acceleration [m/s^2] -- requestor port.
    pub gravity: UniRequestor<(), f64>,

    /// Position [rad] -- internal state.
    pos: Radians,
    /// Previous velocity [rad/s] -- internal state.
    prev_vel: Radians,
    /// Previous acceleration [rad/s^2] -- internal state.
    prev_acc: Radians,
    /// Time of last position update -- internal state.
    last_position_update: MonotonicTime,
    /// Mass on the end of pendulum [kg] -- constant.
    mass: f64,
    /// Length of the pendulum -- constant.
    length: f64,
}

#[Model]
impl Pendulum {
    /// Creates a new pendulum model.
    pub fn new(
        initial_position: Radians,
        mass: f64,
        length: f64,
        gravity_requestor: UniRequestor<(), f64>,
    ) -> Self {
        assert!(mass > 0.0);
        assert!(length > 0.0);
        Self {
            position: Default::default(),
            velocity: Default::default(),
            acceleration: Default::default(),
            energy: Default::default(),
            gravity: gravity_requestor,
            pos: initial_position.normalize_two_pi(),
            prev_vel: Radians::new(0.0),
            prev_acc: Radians::new(0.0),
            last_position_update: MonotonicTime::EPOCH,
            mass,
            length,
        }
    }

    /// Broadcasts the initial position of the pendulum.
    #[nexosim(init)]
    async fn init(&mut self, cx: &Context<Self>) {
        self.position.send(self.pos.to_degrees().value()).await;
        self.velocity.send(self.prev_vel.value()).await;
        self.acceleration.send(self.prev_acc.value()).await;
        self.last_position_update = cx.time();
    }

    /// Torque applied at the center of rotation [Nm].
    ///
    /// Calculates pendulum position. Assumes 'elapsed_time' is small.
    pub async fn torque_in(&mut self, torque: f64, cx: &Context<Self>) {
        // Request gravitational acceleration value.
        let g = self.gravity.send(()).await;
        // Calculate time since last update.
        let now = cx.time();
        let elapsed_time = now.duration_since(self.last_position_update).as_secs_f64();
        // Current velocity calculated based on acceleration during last period.
        let vel = self.prev_vel + self.prev_acc * elapsed_time;
        // Integration of velocity over time to get position change.
        self.pos += vel * elapsed_time;
        // Convert position to [0; 2.0*PI] range.
        self.pos = self.pos.normalize_two_pi();
        // Saves velocity for next iteration.
        self.prev_vel = vel;
        // Saves time for next iteration.
        self.last_position_update = now;
        // Calculates current acceleration to use at next iteration.
        let gravity_torque = self.mass * g * self.pos.sin() * self.length;
        let rotational_inertia = self.mass * self.length * self.length;
        let mut acceleration = Radians::new((torque - gravity_torque) / rotational_inertia);
        // Adds friction.
        let normal_force = self.mass * g * self.pos.cos();
        let centrifugal_force =
            self.mass * self.prev_vel.value() * self.prev_vel.value() * self.length;
        let bearing_radius = 0.02;
        let friction_coefficient = 0.1; // for plain bearing
        let friction_torque =
            (normal_force + centrifugal_force) * friction_coefficient * bearing_radius;
        acceleration +=
            Radians::new(-self.prev_vel.value().signum() * friction_torque) / rotational_inertia;
        self.prev_acc = acceleration;

        // Sends position.
        self.position.send(self.pos.to_degrees().value()).await;
        // Sends velocity.
        self.velocity.send(self.prev_vel.value()).await;
        // Sends acceleration.
        self.acceleration.send(self.prev_acc.value()).await;
        // Calculates and sends energy.
        let kinetic_energy = 0.5
            * self.mass
            * self.prev_vel.value()
            * self.prev_vel.value()
            * self.length
            * self.length;
        let potential_energy = self.mass * g * self.length * (1.0 - self.pos.cos());
        self.energy.send(kinetic_energy + potential_energy).await;
    }
}

/// Controller utilizing PID algorithm.
#[derive(Serialize, Deserialize)]
pub struct Controller {
    /// Torque applied to the pendulum -- output port.
    pub torque_out: Output<f64>,
    /// Setpoint for regulator [degrees] -- output port.
    pub setpoint_out: Output<f64>,

    /// Position of the pendulum [rad] -- internal state.
    pos: Radians,
    /// Setpoint for regulator [rad] -- internal state.
    setpoint: Radians,
    /// Period of the control loop [s] -- constant.
    period: f64,
    /// Implementation of PID controller.
    pid: PidController,
}

#[Model]
impl Controller {
    /// Creates a new servo controller.
    pub fn new(
        period: f64,
        proportional_gain: f64,
        integral_gain: f64,
        derivative_gain: f64,
        initial_setpoint: Degrees,
    ) -> Self {
        assert!(period > 0.0);
        let pid = PidController::new(proportional_gain, integral_gain, derivative_gain);

        Self {
            torque_out: Default::default(),
            setpoint_out: Default::default(),
            pos: Radians::new(0.0),
            setpoint: initial_setpoint.to_radians(),
            period,
            pid,
        }
    }

    #[nexosim(init)]
    async fn init(&mut self, cx: &Context<Self>) {
        // Broadcast initial setpoint
        self.setpoint_out
            .send(self.setpoint.to_degrees().value())
            .await;
        // Schedule logic iterations
        let period = Duration::from_secs_f64(self.period);
        cx.schedule_periodic_event(
            period,
            period,
            schedulable!(Self::update_controller_logic),
            (),
        )
        .unwrap();
    }

    /// Sets the position.
    pub async fn position_in(&mut self, position: f64) {
        // Convert position to [0; 2.0*PI] range.
        self.pos = Degrees::new(position).to_radians().normalize_two_pi();
    }

    /// Sets the setpoint
    pub async fn setpoint_in(&mut self, angle: f64) {
        self.setpoint = Degrees::new(angle).to_radians().normalize_two_pi();
        self.setpoint_out
            .send(self.setpoint.to_degrees().value())
            .await;
    }

    /// Sends torque value.
    #[nexosim(schedulable)]
    async fn update_controller_logic(&mut self, _: ()) {
        // Normalized error.
        let mut error = self.setpoint - self.pos;
        let alt_error = -error.value().signum() * (Radians::new(2.0 * PI) - error.abs());
        if alt_error.abs() < error.abs() {
            error = alt_error;
        }
        // Calculating control value using PID regulator.
        let torque = self.pid.update(error.value(), self.period);
        // Sending torque value.
        self.torque_out.send(torque).await;
    }
}

/// Model of environment in which the pendulum is located.
#[derive(Serialize, Deserialize)]
pub struct Environment;

#[Model]
impl Environment {
    /// Returns the value of gravitational acceleration.
    pub async fn gravity(&mut self) -> f64 {
        9.80665
    }
}

/// Bench function for the simulation server. Takes a channel to send event readers to allow access to output ports from within Rust.
fn pendulum_bench(
    viz_tx: mpsc::Sender<EventSlotReader<f64>>,
) -> impl Fn(Option<f64>) -> Result<SimInit, Box<dyn Error>> {
    move |_cfg| {
        // Parameters
        let period = 0.01;
        let proportional_gain = 30.0;
        let integral_gain = 15.0;
        let derivative_gain = 10.0;
        let initial_setpoint = Degrees::new(90.0);

        // Models
        let environment = Environment;
        let environment_mbox = Mailbox::new();
        let gravity_requestor = UniRequestor::new(Environment::gravity, &environment_mbox);
        let mut pendulum = Pendulum::new(Radians::new(PI / 2.0), 1.0, 1.0, gravity_requestor);
        let pendulum_mbox = Mailbox::new();
        let mut controller = Controller::new(
            period,
            proportional_gain,
            integral_gain,
            derivative_gain,
            initial_setpoint,
        );
        let controller_mbox = Mailbox::new();

        // Connections
        pendulum
            .position
            .connect(Controller::position_in, &controller_mbox);
        controller
            .torque_out
            .connect(Pendulum::torque_in, &pendulum_mbox);

        let mut bench = SimInit::new();

        // Connections to allow access to output ports from within Rust.
        let (position_sink, position_reader) = event_slot(SinkState::Enabled);
        pendulum.position.connect_sink(position_sink);
        viz_tx.send(position_reader).unwrap();

        let (position_sink, position_reader) = event_slot(SinkState::Enabled);
        pendulum.position.connect_sink(position_sink);
        viz_tx.send(position_reader).unwrap();

        let (setpoint_sink, setpoint_reader) = event_slot(SinkState::Enabled);
        controller.setpoint_out.connect_sink(setpoint_sink);
        viz_tx.send(setpoint_reader).unwrap();

        let (torque_sink, torque_reader) = event_slot(SinkState::Enabled);
        controller.torque_out.connect_sink(torque_sink);
        viz_tx.send(torque_reader).unwrap();

        let (velocity_sink, velocity_reader) = event_slot(SinkState::Enabled);
        pendulum.velocity.connect_sink(velocity_sink);
        viz_tx.send(velocity_reader).unwrap();

        let (acceleration_sink, acceleration_reader) = event_slot(SinkState::Enabled);
        pendulum.acceleration.connect_sink(acceleration_sink);
        viz_tx.send(acceleration_reader).unwrap();

        let (energy_sink, energy_reader) = event_slot(SinkState::Enabled);
        pendulum.energy.connect_sink(energy_sink);
        viz_tx.send(energy_reader).unwrap();

        // Connections to allow access to output ports from Python client.
        let position = event_queue_endpoint(&mut bench, SinkState::Enabled, "position").unwrap();
        pendulum.position.connect_sink(position);

        let setpoint = event_queue_endpoint(&mut bench, SinkState::Enabled, "setpoint").unwrap();
        controller.setpoint_out.connect_sink(setpoint);

        // Connection to allow sending setpoint events from Python client.
        let _ = EventSource::new()
            .connect(Controller::setpoint_in, &controller_mbox)
            .bind_endpoint(&mut bench, "setpoint");

        // Adding models to the simulation.
        let sim = bench
            .add_model(pendulum, pendulum_mbox, "pendulum")
            .add_model(controller, controller_mbox, "controller")
            .add_model(environment, environment_mbox, "environment");
        Ok(sim)
    }
}

fn main() -> Result<(), SimulationError> {
    // Channel to send event readers.
    let (viz_tx, viz_rx) = mpsc::channel();

    // Server on a background thread (it blocks).
    std::thread::spawn(move || {
        server::run(pendulum_bench(viz_tx), "0.0.0.0:41633".parse().unwrap()).unwrap()
    });

    // Code below is executed after the client connects to the server and the simulation is initialized.

    // Receives event readers from the server thread in order of sending.
    let mut position_for_thread = viz_rx.recv().unwrap();
    let mut position = viz_rx.recv().unwrap();
    let mut setpoint = viz_rx.recv().unwrap();
    let mut torque = viz_rx.recv().unwrap();
    let mut velocity = viz_rx.recv().unwrap();
    let mut acceleration = viz_rx.recv().unwrap();
    let mut energy = viz_rx.recv().unwrap();

    // Thread sending position over UDP.
    std::thread::spawn(move || {
        let socket = UdpSocket::bind("0.0.0.0:0").unwrap();
        loop {
            let position = position_for_thread.read();
            if let Some(position) = position {
                let bytes = position.to_be_bytes();
                let _ = socket.send_to(&bytes, "127.0.0.1:8080");
            };
        }
    });

    // Visualization using raylib with debug information.
    let (mut rl, thread) = raylib::init()
        .size(640 * 2, 480 * 2)
        .title("Pendulum")
        .build();
    let x0 = 640;
    let y0 = 480;
    let len = 400.0;
    let mut angle = Degrees::new(0.0);
    let mut setpoint_value = Degrees::new(0.0);
    let mut torque_value = 0.0;
    let mut velocity_value = 0.0;
    let mut acceleration_value = 0.0;
    let mut energy_value = 0.0;

    while !rl.window_should_close() {
        angle = Degrees::new(position.try_read().unwrap_or(angle.value()));
        setpoint_value = Degrees::new(setpoint.try_read().unwrap_or(setpoint_value.value()));
        torque_value = torque.try_read().unwrap_or(torque_value);
        velocity_value = velocity.try_read().unwrap_or(velocity_value);
        acceleration_value = acceleration.try_read().unwrap_or(acceleration_value);
        energy_value = energy.try_read().unwrap_or(energy_value);
        let mut d = rl.begin_drawing(&thread);
        d.clear_background(Color::RAYWHITE);
        d.draw_text(
            &format!("Angle: {:.2} degrees", angle.value()),
            10,
            10,
            20,
            Color::BLACK,
        );
        d.draw_text(
            &format!("Setpoint: {:.2} degrees", setpoint_value.value()),
            10,
            40,
            20,
            Color::BLACK,
        );
        d.draw_text(
            &format!("Torque: {:.2} Nm", torque_value),
            10,
            70,
            20,
            Color::BLACK,
        );
        d.draw_text(
            &format!("Velocity: {:.2} rad/s", velocity_value),
            10,
            100,
            20,
            Color::BLACK,
        );
        d.draw_text(
            &format!("Acceleration: {:.2} rad/s²", acceleration_value),
            10,
            130,
            20,
            Color::BLACK,
        );
        d.draw_text(
            &format!("Energy: {:.2} J", energy_value),
            10,
            160,
            20,
            Color::BLACK,
        );
        let x = x0 + (angle.sin() * len).round() as i32;
        let y = y0 + (angle.cos() * len).round() as i32;
        d.draw_line(x0, y0, x, y, Color::BLACK);
        d.draw_circle(x, y, 20.0, Color::GOLD);
    }

    Ok(())
}

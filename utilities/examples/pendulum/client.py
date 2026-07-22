import time
import timeit
import matplotlib.pyplot as plt

from nexosim import Simulation
from nexosim.time import Duration

# Plot configuration
plt.ion() 
fig, ax = plt.subplots(figsize=(10, 5))

times = []
positions = []
current_time_s = 0.0

(line,) = ax.plot([], [], "b-", label="Position")
ax.set_title("Real-time Pendulum Position Plot")
ax.set_xlabel("Simulation Time [s]")
ax.set_ylabel("Position [rad]")
ax.grid(True)
ax.legend()


def step_and_pos(sim, step_duration):
    global current_time_s

    # Simulation step
    sim.step_until(step_duration)

    # Update current time
    step_in_seconds = step_duration.nanos / 1e9 + step_duration.secs
    current_time_s += step_in_seconds

    # Read position events
    pos_events = sim.try_read_events("position")

    # If there are position events, take the latest one and update the plot
    if pos_events:
        latest_pos = pos_events[-1]
        print(f"t={current_time_s:.4f}s | pos={latest_pos}")

        times.append(current_time_s)
        positions.append(latest_pos)

        line.set_xdata(times)
        line.set_ydata(positions)
        ax.relim()
        ax.autoscale_view()
        fig.canvas.draw()
        fig.canvas.flush_events()


with Simulation("0.0.0.0:41633") as sim:
    sim.build()
    sim.init()
    sim.process_event("setpoint", 270.0)

    print("Initial position events:", sim.try_read_events("position"))

    duration = Duration.create(microseconds=16)

    try:
        while True:
            # Measure the time taken for a simulation step and adjust the duration accordingly
            d = timeit.timeit(lambda: step_and_pos(sim, duration), number=1)
            duration = Duration.create(nanoseconds=int(d * 1e9))

    except KeyboardInterrupt:
        plt.ioff()
        plt.show()


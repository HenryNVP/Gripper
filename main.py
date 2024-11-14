import numpy as np
import matplotlib.pyplot as plt
from fuzzy_controller import FuzzyController

if __name__ == "__main__":
    controller = FuzzyController()
    sampling_time = 0.1  # Time step in seconds

    time_current = 0
    time_history = []
    time_history.append(time_current)

    force_current = 0.4  # [kg]
    force_derivative = 0
    force_output_history = []
    force_output_history.append(force_current)

    # The desired force is a square wave
    force_desired_history = []
    force_desired_history.append(force_current)
    T = 10  # Period of the square wave in seconds
    lower = 0.2  # Lower value of the square wave
    upper = 0.6  # Upper value of the square wave

    while time_current < 4 * T:
        force_desired = lower + (upper - lower) * 0.5 * \
            (1 + np.sign(np.sin(2 * np.pi * time_current / T)))
        force_error = force_desired - force_current
        actuator_output = controller.fuzzy_controller(
            force_error, force_derivative)
        force_desired_history.append(force_desired)

        force_current = force_current * (1 + actuator_output)
        force_derivative = actuator_output / sampling_time
        force_output_history.append(force_current)

        time_current = time_current + sampling_time
        time_history.append(time_current)

    # Plot force output
    plt.figure(figsize=(12, 6))
    plt.plot(time_history, force_output_history,
             label="Output force", color='g')
    plt.plot(time_history, force_desired_history,
             label="Desired force", color='b')
    plt.title('Simulation of Fuzzy Force Controller')
    plt.xlabel('Time (s)')
    plt.ylabel('Force (kg)')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

# Force Fuzzy Control Simulation

## Overview
This project implements a force control simulation for a robotic robotic hand using fuzzy logic based on theory in [1]. The simulation models a system where the force applied by the actuator needs to be adjusted based on the error in the current force and its derivative. Using fuzzy logic offers a reliable approach to control grasping forc of the gripper.

## Features
- Fuzzy Logic Controller: Designs membership function to get the result close to theory.
- Dynamic Simulation: Simulates the force control over time, plotting results for analysis.

## Prerequisites
Install the following required Python packages:
   ```bash
   pip install numpy scipy matplotlib scikit-fuzzy networkx
   ```

## Usage
Run the simulation script to visualize the force control output:
```bash
python main.py
```

## Reference
[1] L. Birglen, Clément Gosselin, and Thierry Laliberté, Underactuated robotic hands. Berlin: Springer, 2008.

## License
This project is licensed under the MIT License.

## Contributions
Contributions are welcome! Feel free to open an issue or submit a pull request for improvements.

---

For further information or questions, please contact [henrynguyen.vp@gmail.com].


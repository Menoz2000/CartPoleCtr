# CartPoleCtr

Un progetto basato su **ROS 2** e **Gazebo** per la simulazione e il controllo di un sistema **Cart-Pole** (pendolo inverso). 

Il progetto integra un ambiente fisico simulato in Gazebo, un nodo per l'applicazione delle forze (Force Simulator) e un nodo di controllo sviluppato in C++ per bilanciare il pendolo.

## Tecnologie Utilizzate
- **Linguaggi:** C++, CMake, Python
- **Middleware:** ROS 2
- **Simulatore:** Gazebo

## Note

launch del nodo gazebo

`ros2 launch gazebo_polecart_ros polecart.launch.py`

launch del nodo force simulator

`.py - ros2 launch force_simulator force_simulator_launch.py`

`.xml - ros2 launch force_simulator simulation_control_force_launch.xml`

launch del nodo cart_pole_controller_am_cpp

`ros2 launch cart_pole_controller_am_cpp cart_pole_controller_am_launch.py`

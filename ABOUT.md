# Dynamic Obstacle Avoidance Using Potential Field Method with LiDAR in a 3D Environment

I gave a robot the ability to sense its surroundings and navigate through them in real time, reaching a target (X, Y) position using nothing but physics and laser light. No pre-built map. No global planner. Just mathematics, a spinning LiDAR, and 50 milliseconds to react.

## 🔹 What is the Potential Field Method?

The Potential Field Method is a classical reactive navigation algorithm inspired by physics. The environment is modeled as a force field consisting of two main components:

*   **Attractive Force** — The goal behaves like a magnet, pulling the robot toward it. The farther the robot is from the goal, the stronger the attractive pull.
*   **Repulsive Force** — Each obstacle generates a repulsive field. When the robot enters an obstacle’s influence radius, it experiences a force pushing it away.

## 🔹 Why Potential Fields Work Well for Dynamic Environments

This is where potential fields do something global planners often struggle with. Because the entire force field is rebuilt from live sensor data every 50 ms, moving obstacles are handled naturally and instantly.

As an obstacle moves, its repulsive field moves with it, causing the robot to avoid where the obstacle is right now—not where it was when a map was last updated. A potential field has no map to become outdated.

Every control cycle answers a simple question:
*Given what my sensors see right now, which direction should I move?*

## 🔹 Simulation Experiment

I built a differential drive robot in a simulated environment and implemented a potential field controller running at 20 Hz. The controller processes:
*   Live LiDAR scan data
*   Wheel odometry

Using this data, it computes attractive and repulsive forces in real time and sends control commands to the robot to navigate toward the goal while avoiding obstacles reactively.

## 🔹 Warehouse Navigation Test

I tested the controller in a complex warehouse simulation with multiple configurations, including:
*   Shelves
*   Pallets
*   Narrow corridors
*   Dense obstacle layouts

The robot successfully navigated through all scenarios and reached the specified (X, Y) target position. For highly dynamic and unpredictable environments, further parameter tuning can improve robustness and stability.

## ⚙️ Key Technologies

*   LiDAR-based perception
*   Reactive navigation
*   Potential field algorithms
*   Differential drive control
*   Real-time obstacle avoidance

> The Potential Field method won't replace RRT or NavFn for complex long-horizon planning—but for real-time reactive avoidance in dynamic environments, nothing beats it for simplicity, speed, and elegance.

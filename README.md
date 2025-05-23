# **Empowering Socially Sensitive Autonomous Vehicles Using Human-plausible Cognitive Encoding**

This repository contains the official codebase for the paper **“Empowering Socially Sensitive Autonomous Vehicles Using Human-plausible Cognitive Encoding”**.

It provides the implementation of ethical and cognitively plausible planning methods for autonomous vehicles in standardized simulation environments.

------



## **📦 Overview of Modules**

### **1. 🧠 Cognitive Encoding Module**

- **Integration**: Embedded within the frenet_planner pipeline (Perception → Encoding → Decision).
- **Function**: Incorporates ethical considerations for different road users, simulating human-like holistic evaluation in driving decision-making.
- **Input Features**: Spatial relationships, relative velocities.
- **Purpose**: Provides interpretable and structured feature support to improve planning behavior and social sensitivity.

------

### **2. 🚗 Trajectory Planning Module (Ethical Frenet Planner [Ethical Frenet Planner Project](https://github.com/TUMFTM/EthicalTrajectoryPlanning))**

- **Function**: Implements a trajectory planner with ethical constraints, supporting safe and reasonable trajectory generation in CommonRoad standard scenarios.
- **Features**: Allows scenario-specific testing and visualizes both the decision-making process and generated trajectories.

---

### **3. 🧾 Logging & Evaluation Tools**

- **Metrics**: Includes cumulative harm score, safety violations, and comfort evaluation.
- **Batch Evaluation**:
  - Supports large-scale testing on 2000+ CommonRoad scenarios.
  - Generates visual metrics such as speed and acceleration curves (see plot_vel_acc.py).
- **Hardware Requirements**:
  - Approx. 200 GB disk space required;
  - Multi-threading and GPU acceleration recommended (e.g., 8× RTX3090).

------

### **4. 🗺️ Scenario Resources (CommonRoad Benchmark Scenarios)**

- **Location**: commonroad-scenarios/
- **Source**: [CommonRoad Project](https://commonroad.in.tum.de/)
- **Purpose**: Provides diverse and high-fidelity simulation environments for testing planning and decision-making generalization capabilities.

------

### **5. 🔁 Active Inference Module**

- **Location**: Active_Inference/
- **Function**: Implements an Active Inference-based decision-making mechanism that can be plugged into the planner to replace the default module.
- **Use Case**: Enables behavior selection driven by information gain, especially under uncertainty in perception.

------

### **6. 🧭 Trajectory Visualization Tool**

- **Location**: visualize_tool/
- **Function**: Visualizes planned trajectories and risks.
<img src="./iShot_2025-04-10_22.39.09.png" alt="App Screenshot" width="50%"/>



## **📮 Request Full Version**

The full version of the code is currently under active maintenance and **will be released soon**.

A preliminary version is **available upon request** from the first author.

If you wish to access the code:

- Please **send an application email** to: lhldlut@163.com
- Clearly **brief your intended use** (academic purposes only)
- ⚠️ **Commercial use is strictly prohibited**
- We will **evaluate your request** (please note that responses may not be immediate — kindly avoid sending duplicate emails)

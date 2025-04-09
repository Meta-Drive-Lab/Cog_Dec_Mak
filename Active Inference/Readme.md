# Free Energy Cost for Trajectory Evaluation

## Function
`calc_free_energy_to_lane_center_custom`

---

## 🔍 Overview

This function calculates the **free energy cost** of a trajectory in a way that is fully consistent with the principles of **Active Inference**. It evaluates how well a trajectory aligns with the agent’s expected latent behavior by measuring deviations from the preferred ref path.

The cost serves as a proxy for variational free energy and can be directly used in planning frameworks based on **Active Inference**, guiding the agent to minimize surprise and maintain consistency with internal beliefs.

---



## 🧩 Signature

```python
def calc_free_energy_to_lane_center_custom(traj, lanelet_network, lambda_param=0.5) -> float
```

**📥 Inputs**

| **Parameter**   | **Type**         | **Description**                                              |
| --------------- | ---------------- | ------------------------------------------------------------ |
| traj            | FrenetTrajectory | Contains the trajectory’s x/y positions over time.           |
| lanelet_network | LaneletNetwork   | Road network object from CommonRoad, includes centerlines, lanelets, etc. |
| lambda_param    | float  | Scaling factor for the final free energy value.              |

------



**📤 Output**

| **Return**  | **Type** | **Description**                                           |
| ----------- | -------- | --------------------------------------------------------- |
| free_energy | float    | Accumulated squared distance to lane centerlines, scaled. |

------



**📎 Example Use**

This function is typically called inside the full trajectory evaluation pipeline:

```
if weights["free_energy"] > 0.0:
    cost_dict["free_energy"] = calc_free_energy_to_lane_center_custom(traj, scenario.lanelet_network)
```

------



**📚 Dependencies**

​	•	numpy

​	•	shapely

​	•	commonroad_dc.lanelet (for lanelet map access)

------


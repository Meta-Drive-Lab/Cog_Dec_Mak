# Free Energy Cost for Trajectory Evaluation

## Function
`calc_free_energy_to_lane_center_custom`

---

## 🔍 Overview

This function implements a **Free Energy minimization principle**, central to the framework of **Active Inference**. In this context, free energy serves as a measure of deviation between the predicted (preferred) latent trajectory — typically aligned with high-probability, stable behaviors — and the actual executed trajectory.



The core idea is that an intelligent agent seeks to minimize this free energy to reduce surprise or uncertainty about its future states. In motion planning, this translates to preferring trajectories that are **consistent with prior beliefs about optimal behavior**, such as staying close to the inferred path of least resistance — often represented by structural priors like ref lane centers.

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


# Multi-DOF-Robotic-Arm-Control-Motion-Planning

**Analytical IK, Jacobian IK, and RRT\*** for Planar and Spatial Manipulators

---

## Overview

This repository implements and contrasts **three fundamentally different approaches** to robotic arm motion generation:

1. **Analytical Inverse Kinematics (IK)** for a planar **2-DOF arm**
2. **Differential (Jacobian-based) IK control** for an **N-DOF planar arm**
3. **Sampling-based motion planning (RRT\*)** for a **7-DOF spatial manipulator** using full kinematic collision checking

The project is intended as a **technical exploration of arm control paradigms**, progressing from closed-form solutions to differential control and finally to global motion planning in high-dimensional configuration spaces.

All implementations are written in **Python**, with real-time visualization using **Matplotlib**.

---

## Reference and Attribution

The analytical two-link inverse kinematics implementation is inspired by and adapted from:

> Atsushi Sakai et al., *PythonRobotics*  
> **Planar Two-Link Inverse Kinematics**  
> https://atsushisakai.github.io/PythonRobotics/modules/7_arm_navigation/planar_two_link_ik.html

This repository **extends that reference** by:
- Improving visualization
- Adding interactive target selection
- Introducing trajectory persistence
- Scaling from 2-DOF to N-DOF Jacobian control
- Extending to 7-DOF motion planning with RRT\*

---

### Features

- Interactive **mouse-based target selection**
- Workspace boundary enforcement
- Elbow-up / elbow-down consistency
- Persistent end-effector trajectory visualization
- Real-time angle and distance overlays

---

### Robot Model

- 7-DOF serial manipulator
- Defined using **Denavit–Hartenberg parameters**
- Forward kinematics computed via homogeneous transformations

---

### RRT\* Algorithm

RRT\* extends RRT by guaranteeing **asymptotic optimality**.

Core steps:
1. Random sampling (with goal bias)
2. Nearest neighbor search
3. Steering with fixed resolution
4. Collision checking along the path
5. Cost-based parent selection
6. Rewiring for cost improvement

---

## Comparison of Control Paradigms

| Method | DOF | Space | Guarantees | Use Case |
|------|-----|------|-----------|---------|
| Analytical IK | 2 | Task | Exact | Low-DOF arms |
| Jacobian IK | N | Task | Local | Redundant control |
| RRT\* | N | Joint | Global optimal | Obstacle avoidance |

---

## Key Takeaways

- **IK is not planning** — Jacobian methods do not avoid obstacles
- **Planning is not control** — RRT\* produces paths, not feedback laws
- High-DOF arms require **hierarchical solutions**:
  - Global planning (RRT\*)
  - Local control (IK / Jacobian)
  - Low-level actuation

This repository demonstrates each layer **explicitly and independently**.

---

## Author

**Aaditya Sakhardande**  
Robotics & Autonomous Systems  
Focus: Manipulator kinematics, motion planning, and intelligent control


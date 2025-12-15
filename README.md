# Multi-DOF-Robotic-Arm-Control-Motion-Planning

# Multi-DOF Robotic Arm Control and Motion Planning  
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

## 1. Planar 2-DOF Arm — Analytical Inverse Kinematics

### Problem Definition

Given a planar 2-link manipulator with link lengths \( l_1, l_2 \), compute joint angles  
\( \theta_1, \theta_2 \) such that the end-effector reaches a desired Cartesian target \( (x, y) \).

---

### Kinematic Model

**Forward Kinematics**
\[
\begin{aligned}
x &= l_1 \cos \theta_1 + l_2 \cos(\theta_1 + \theta_2) \\
y &= l_1 \sin \theta_1 + l_2 \sin(\theta_1 + \theta_2)
\end{aligned}
\]

**Inverse Kinematics (Law of Cosines)**

\[
\cos \theta_2 =
\frac{x^2 + y^2 - l_1^2 - l_2^2}{2 l_1 l_2}
\]

\[
\theta_1 = \tan^{-1}\left(\frac{y}{x}\right)
- \tan^{-1}\left(\frac{l_2 \sin \theta_2}{l_1 + l_2 \cos \theta_2}\right)
\]

---

### Control Strategy

Rather than directly jumping to the IK solution, the system uses a **proportional feedback controller in joint space**:

\[
\dot{\theta} = K_p \cdot (\theta_{\text{goal}} - \theta)
\]

This ensures:
- Smooth convergence
- Stability near singularities
- Continuous visualization

Angular wrap-around is handled using modular arithmetic.

---

### Features

- Interactive **mouse-based target selection**
- Workspace boundary enforcement
- Elbow-up / elbow-down consistency
- Persistent end-effector trajectory visualization
- Real-time angle and distance overlays

---

## 2. Planar N-DOF Arm — Jacobian-Based Inverse Kinematics

### Motivation

Analytical IK becomes intractable beyond low DOF.  
For redundant manipulators, **differential inverse kinematics** provides a scalable alternative.

---

### Forward Kinematics (Serial Chain)

For an N-DOF planar arm:
\[
\begin{aligned}
x &= \sum_{i=1}^{N} l_i \cos\left(\sum_{j=1}^{i} \theta_j\right) \\
y &= \sum_{i=1}^{N} l_i \sin\left(\sum_{j=1}^{i} \theta_j\right)
\end{aligned}
\]

---

### Jacobian Derivation

The Jacobian maps joint velocities to end-effector velocity:

\[
\dot{x} = J(\theta)\dot{\theta}
\]

Each column \( J_i \) corresponds to the partial derivative of the end-effector position
with respect to \( \theta_i \):

\[
J =
\begin{bmatrix}
-\sum_{j=i}^{N} l_j \sin(\sum_{k=1}^{j} \theta_k) \\
\sum_{j=i}^{N} l_j \cos(\sum_{k=1}^{j} \theta_k)
\end{bmatrix}
\]

---

### Damped Least Squares (DLS) Inversion

To avoid instability near singularities, the **Levenberg–Marquardt** formulation is used:

\[
\dot{\theta} =
J^T (J J^T + \lambda^2 I)^{-1} e
\]

where:
- \( e = x_{\text{goal}} - x_{\text{current}} \)
- \( \lambda \) is the damping coefficient

---

## 3. 7-DOF Spatial Arm — RRT* Motion Planning

### Problem Statement

This module implements **RRT\*** (Rapidly-Exploring Random Tree Star) for planning a
**collision-free joint-space trajectory** for a **7-DOF robotic manipulator**.

Given:
- Start configuration:  
  \[
  q_{\text{start}} \in \mathbb{R}^7
  \]
- Goal configuration:  
  \[
  q_{\text{goal}} \in \mathbb{R}^7
  \]

the planner computes an **optimal joint-space path** while avoiding obstacles in the
3D workspace.

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





## Repository Structure


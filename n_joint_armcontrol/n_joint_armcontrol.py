"""
Inverse kinematics of a n-joint arm (visual improvements)
Left-click the plot to set the goal position of the end effector

Author: Aaditya Sakhardande
"""
import numpy as np
import matplotlib.pyplot as plt
import math

# ======================
# Parameters
# ======================
dt = 0.009
Kp = 9
n_joints = 8
link_lengths = np.ones(n_joints) * 0.4

x_goal, y_goal = 2.0, 0.0
show_animation = True

if show_animation:
    plt.ion()

trail = []


# ======================
# Forward Kinematics
# ======================
def forward_kinematics(thetas):
    x, y = 0.0, 0.0
    angle = 0.0
    points = [(0.0, 0.0)]

    for i in range(n_joints):
        angle += thetas[i]
        x += link_lengths[i] * np.cos(angle)
        y += link_lengths[i] * np.sin(angle)
        points.append((x, y))

    return np.array(points)


# ======================
# Jacobian
# ======================
def compute_jacobian(thetas):
    J = np.zeros((2, n_joints))
    angle = 0.0

    for i in range(n_joints):
        angle += thetas[i]
        dx = dy = 0.0

        for j in range(i, n_joints):
            a = sum(thetas[:j+1])
            dx -= link_lengths[j] * np.sin(a)
            dy += link_lengths[j] * np.cos(a)

        J[0, i] = dx
        J[1, i] = dy

    return J


# ======================
# IK Loop
# ======================
def six_joint_arm():
    global x_goal, y_goal, trail

    thetas = np.zeros(n_joints)

    while True:
        points = forward_kinematics(thetas)
        wrist = points[-1]

        error = np.array([x_goal - wrist[0], y_goal - wrist[1]])

        if np.linalg.norm(error) < 0.01:
            continue

        lambda_ = 0.1
        J = compute_jacobian(thetas)
        JT = J.T
        dtheta = Kp * JT @ np.linalg.inv(J @ JT + lambda_**2 * np.eye(2)) @ error

        thetas += dtheta * dt

        plot_arm(points, wrist)

        trail.append(tuple(wrist))
        if len(trail) > 300:
            trail = trail[-300:]


# ======================
# Plotting
# ======================
def plot_arm(points, wrist):
    plt.cla()
    ax = plt.gca()
    ax.set_aspect('equal')

    # links
    xs, ys = zip(*points)
    plt.plot(xs, ys, linewidth=4)
    plt.scatter(xs, ys, s=50)

    # target
    plt.scatter(x_goal, y_goal, marker='*', s=150)
    plt.plot([wrist[0], x_goal], [wrist[1], y_goal], '--')

    # trail
    if len(trail) > 1:
        tx, ty = zip(*trail)
        plt.plot(tx, ty, alpha=0.5)

    reach = sum(link_lengths)
    plt.xlim(-reach, reach)
    plt.ylim(-reach, reach)
    plt.grid(alpha=0.3)
    plt.title("6-DOF Planar Arm — Jacobian IK")

    plt.pause(dt)


# ======================
# Mouse Input
# ======================
def click(event):
    global x_goal, y_goal, trail
    if event.xdata is None or event.ydata is None:
        return
    x_goal = event.xdata
    y_goal = event.ydata
    trail = []


# ======================
# Main
# ======================
def main():
    fig = plt.figure(figsize=(6, 6))
    fig.canvas.mpl_connect("button_press_event", click)
    fig.canvas.mpl_connect(
        'key_release_event',
        lambda event: exit(0) if event.key == 'escape' else None
    )
    six_joint_arm()


if __name__ == "__main__":
    main()

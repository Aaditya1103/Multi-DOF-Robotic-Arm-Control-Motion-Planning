"""
Inverse kinematics of a two-joint arm (visual improvements)
Left-click the plot to set the goal position of the end effector

Author: Daniel Ingram (daniel-s-ingram)
Modified visualization: Aaditya Sakhardande
"""

import matplotlib.pyplot as plt
import numpy as np
import math
from utils.angle import angle_mod

# Simulation parameters
Kp = 15
dt = 0.01

# Link lengths
l1 = l2 = 1

# Set initial goal position to the initial end-effector position
x = 2
y = 0

show_animation = True

if show_animation:
    plt.ion()

# persistent trail for end-effector
trail = []


def two_joint_arm(GOAL_TH=0.0, theta1=0.0, theta2=0.0):
    """
    Computes the inverse kinematics for a planar 2DOF arm
    When out of bounds, rewrite x and y with last correct values
    """
    global x, y, trail
    x_prev, y_prev = None, None
    while True:
        try:
            if x is not None and y is not None:
                x_prev = x
                y_prev = y

            # If target outside workspace, clamp to boundary by setting theta2_goal = 0
            if np.hypot(x, y) > (l1 + l2):
                theta2_goal = 0.0
            else:
                # Law of cosines: cos(theta2) = (r^2 - l1^2 - l2^2) / (2 l1 l2)
                val = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
                # numerical safety clamp
                val = np.clip(val, -1.0, 1.0)
                theta2_goal = np.arccos(val)

            tmp = math.atan2(l2 * np.sin(theta2_goal),
                             (l1 + l2 * np.cos(theta2_goal)))
            theta1_goal = math.atan2(y, x) - tmp

            # choose elbow-up / elbow-down consistently
            if theta1_goal < 0:
                theta2_goal = -theta2_goal
                tmp = math.atan2(l2 * np.sin(theta2_goal),
                                 (l1 + l2 * np.cos(theta2_goal)))
                theta1_goal = math.atan2(y, x) - tmp

            theta1 = theta1 + Kp * ang_diff(theta1_goal, theta1) * dt
            theta2 = theta2 + Kp * ang_diff(theta2_goal, theta2) * dt

        except ValueError as e:
            print("Unreachable goal: " + str(e))
        except TypeError:
            # if click yields None or invalid, revert to previous valid target
            x = x_prev
            y = y_prev

        wrist = plot_arm(theta1, theta2, x, y)

        # update trail
        if wrist is not None:
            trail.append(tuple(wrist))
            # cap trail length
            if len(trail) > 200:
                trail = trail[-200:]

        # check goal
        d2goal = None
        if x is not None and y is not None and wrist is not None:
            d2goal = np.hypot(wrist[0] - x, wrist[1] - y)

        if d2goal is not None and abs(d2goal) < GOAL_TH and x is not None:
            return theta1, theta2


def plot_arm(theta1, theta2, target_x, target_y):  # pragma: no cover
    shoulder = np.array([0.0, 0.0])
    elbow = shoulder + np.array([l1 * np.cos(theta1), l1 * np.sin(theta1)])
    wrist = elbow + np.array([l2 * np.cos(theta1 + theta2),
                              l2 * np.sin(theta1 + theta2)])

    if show_animation:
        plt.cla()
        ax = plt.gca()
        ax.set_aspect('equal', 'box')

        # workspace circle
        workspace = plt.Circle((0, 0), l1 + l2, fill=False, linestyle=':', alpha=0.5)
        ax.add_artist(workspace)

        # draw links (thicker)
        plt.plot([shoulder[0], elbow[0]], [shoulder[1], elbow[1]], linewidth=4, solid_capstyle='round')
        plt.plot([elbow[0], wrist[0]], [elbow[1], wrist[1]], linewidth=4, solid_capstyle='round')

        # joints as filled circles
        plt.scatter(shoulder[0], shoulder[1], s=80, marker='o')
        plt.scatter(elbow[0], elbow[1], s=80, marker='o')
        plt.scatter(wrist[0], wrist[1], s=60, marker='o')

        # show target and dashed line to it
        if target_x is not None and target_y is not None:
            plt.plot([wrist[0], target_x], [wrist[1], target_y], linestyle='--', linewidth=1.5)
            plt.scatter([target_x], [target_y], marker='*', s=120)

        # draw trail of wrist positions
        if len(trail) > 1:
            tx = [p[0] for p in trail]
            ty = [p[1] for p in trail]
            plt.plot(tx, ty, linewidth=1, alpha=0.7)

        # text overlay: angles and distance to goal
        theta1_deg = math.degrees(theta1)
        theta2_deg = math.degrees(theta2)
        dist_text = ''
        if target_x is not None and target_y is not None:
            d2goal = np.hypot(wrist[0] - target_x, wrist[1] - target_y)
            dist_text = f"dist -> {d2goal:.3f}"
        plt.text(-1.95, 1.85, f"θ1 = {theta1_deg:.1f}°\nθ2 = {theta2_deg:.1f}°\n{dist_text}", fontsize=9,
                 bbox=dict(facecolor='white', alpha=0.7, edgecolor='none'))

        plt.xlim(-2.05, 2.05)
        plt.ylim(-2.05, 2.05)
        plt.xlabel("x")
        plt.ylabel("y")
        plt.title("2-DOF planar arm — left click to set target, Esc to exit")

        plt.grid(alpha=0.3)

        plt.show()
        plt.pause(dt)

    return wrist


def ang_diff(theta1, theta2):
    # Returns the difference between two angles in the range -pi to +pi
    return angle_mod(theta1 - theta2)


def click(event):  # pragma: no cover
    global x, y, trail
    # Ignore clicks outside axes
    if event.xdata is None or event.ydata is None:
        return
    x = event.xdata
    y = event.ydata
    # reset trail when setting a new explicit goal (optional)
    trail = []


def animation():
    from random import random
    global x, y
    theta1 = theta2 = 0.0
    for i in range(5):
        x = 2.0 * random() - 1.0
        y = 2.0 * random() - 1.0
        theta1, theta2 = two_joint_arm(
            GOAL_TH=0.01, theta1=theta1, theta2=theta2)


def main():  # pragma: no cover
    fig = plt.figure(figsize=(6, 6))
    fig.canvas.mpl_connect("button_press_event", click)
    # for stopping simulation with the esc key.
    fig.canvas.mpl_connect('key_release_event', lambda event: [
                           exit(0) if event.key == 'escape' else None])
    two_joint_arm()


if __name__ == "__main__":
    # animation()
    main()


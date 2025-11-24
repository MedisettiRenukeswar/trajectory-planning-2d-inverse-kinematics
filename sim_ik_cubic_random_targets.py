import math
import random
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from arm_math import forward_kinematics, inverse_kinematics_2link, L1, L2
from cubic_trajectory import compute_cubic_coeffs, eval_cubic

# ---------- Simulation parameters ----------
DT = 0.05              # time step per frame (s)
STEPS = 800            # total frames for interactive run
SEGMENT_DURATION = 2.0 # seconds for each trajectory segment
TARGET_HOLD_FRAMES = 20  # frames to hold at target before choosing new one


def random_reachable_target():
    """
    Sample a random reachable (x, y) for the 2-link arm.
    """
    margin = 0.1
    r_min = abs(L1 - L2) + margin
    r_max = (L1 + L2) - margin

    radius = random.uniform(r_min, r_max)
    angle = random.uniform(-math.pi, math.pi)

    x = radius * math.cos(angle)
    y = radius * math.sin(angle)
    return x, y


# ---------- Matplotlib setup ----------
fig, ax = plt.subplots()
ax.set_aspect("equal", "box")
ax.set_xlim(-3, 3)
ax.set_ylim(-3, 3)
ax.set_title("2-Link Arm – IK + Cubic Trajectory to Random Targets")

base_plot, = ax.plot([], [], "ko", markersize=8)
elbow_plot, = ax.plot([], [], "bo", markersize=6)
end_plot, = ax.plot([], [], "ro", markersize=6)
link1_line, = ax.plot([], [], "b-", linewidth=3)
link2_line, = ax.plot([], [], "r-", linewidth=3)
target_plot, = ax.plot([], [], "gx", markersize=8, label="Target")


# ---------- Trajectory state ----------
theta1_curr = 0.0
theta2_curr = 0.0

theta1_coeffs = None
theta2_coeffs = None
segment_time = 0.0
segment_active = False
hold_counter = 0

target_x = 0.0
target_y = 0.0


def start_new_segment():
    """
    Choose a new random target, compute IK, and set up a cubic trajectory
    from current joint angles to target joint angles.
    """
    global theta1_curr, theta2_curr
    global theta1_coeffs, theta2_coeffs
    global segment_time, segment_active
    global target_x, target_y, hold_counter

    # Find a valid target + IK
    for _ in range(20):
        tx, ty = random_reachable_target()
        ik = inverse_kinematics_2link(tx, ty, elbow_up=True)
        if ik is not None:
            t1_target, t2_target = ik
            target_x, target_y = tx, ty
            break
    else:
        # If no valid target found, keep old one
        print("WARNING: Could not find valid target, keeping previous.")
        segment_active = False
        return

    # Compute cubic coefficients for each joint (zero vel at start & end)
    theta1_coeffs = compute_cubic_coeffs(theta1_curr, t1_target, v0=0.0, vT=0.0, T=SEGMENT_DURATION)
    theta2_coeffs = compute_cubic_coeffs(theta2_curr, t2_target, v0=0.0, vT=0.0, T=SEGMENT_DURATION)

    segment_time = 0.0
    segment_active = True
    hold_counter = 0

    print(
        f"New target: ({target_x:.2f}, {target_y:.2f}) "
        f"→ IK (deg)=({math.degrees(t1_target):.1f}, {math.degrees(t2_target):.1f})"
    )


def init():
    start_new_segment()
    base_plot.set_data([0], [0])
    target_plot.set_data([target_x], [target_y])
    return base_plot, elbow_plot, end_plot, link1_line, link2_line, target_plot


def update(frame_idx):
    global theta1_curr, theta2_curr
    global segment_time, segment_active, hold_counter

    # If we have an active cubic segment, advance along it
    if segment_active and theta1_coeffs is not None and theta2_coeffs is not None:
        segment_time += DT
        t = min(segment_time, SEGMENT_DURATION)

        theta1_curr, _ = eval_cubic(theta1_coeffs, t)
        theta2_curr, _ = eval_cubic(theta2_coeffs, t)

        # If trajectory is finished, mark as inactive and start holding
        if segment_time >= SEGMENT_DURATION:
            segment_active = False
            hold_counter = 0
    else:
        # We are at target, hold for a bit, then pick a new one
        hold_counter += 1
        if hold_counter > TARGET_HOLD_FRAMES:
            start_new_segment()

    # FK for visualization
    (x1, y1), (x2, y2) = forward_kinematics(theta1_curr, theta2_curr)

    base_plot.set_data([0], [0])
    elbow_plot.set_data([x1], [y1])
    end_plot.set_data([x2], [y2])

    link1_line.set_data([0, x1], [0, y1])
    link2_line.set_data([x1, x2], [y1, y2])

    target_plot.set_data([target_x], [target_y])

    return base_plot, elbow_plot, end_plot, link1_line, link2_line, target_plot


if __name__ == "__main__":
    ax.legend(loc="upper left")

    # Normal interactive animation (for running & understanding)
    anim = FuncAnimation(fig, update, init_func=init,
                         frames=STEPS, interval=50, blit=True)

    # --------- CONFIG: ONLY TURN THIS ON WHEN YOU WANT TO EXPORT GIF ----------
    SAVE_GIF = False  # <- set to True ONLY when you want to save

    if SAVE_GIF:
        import os
        from matplotlib.animation import PillowWriter

        os.makedirs("assets", exist_ok=True)

        print("Saving GIF... this may take a few seconds.")
        # Use a SHORTER animation for GIF export to avoid huge files
        gif_frames = 250  # <= much smaller than STEPS
        gif_anim = FuncAnimation(fig, update, init_func=init,
                                 frames=gif_frames, interval=50, blit=True)

        gif_writer = PillowWriter(fps=8)  # fewer FPS → fewer frames to encode
        gif_anim.save("assets/sim_ik_cubic_random_targets3.gif", writer=gif_writer, dpi=70)
        print("Saved GIF to assets/sim_ik_cubic_random_targets.gif")
    else:
        plt.show()

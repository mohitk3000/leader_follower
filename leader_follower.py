import pygame
import numpy as np
import math
import sys
import random
from PIL import Image
import os

# ----------------------------
# PARAMETERS
# ----------------------------
WIDTH, HEIGHT = 800, 600

DT = 0.05
SAMPLE_TIME = 0.5

LEADER_V = 80.0
LEADER_W = -0.2

NOISE_STD = 2.0
MAX_TRAIL = 500

EXPORT_GIF = True

# ----------------------------
# INITIAL STATES
# ----------------------------
leader = np.array([400.0, 300.0, 0.0])
follower = np.array([100.0, 100.0, 0.0])

leader_history = []
follower_history = []
sampled_targets = []

frames = []

time = 0.0
last_sample_time = -SAMPLE_TIME

target = None
target_start_time = 0

# ----------------------------
# PYGAME SETUP
# ----------------------------
pygame.init()

screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Leader-Follower Simulation")

clock = pygame.time.Clock()

# ----------------------------
# FUNCTIONS
# ----------------------------
def draw_robot(x, y, theta, color):

    size = 12

    pts = [
        (
            x + size * math.cos(theta),
            y + size * math.sin(theta)
        ),

        (
            x + size * math.cos(theta + 2.5),
            y + size * math.sin(theta + 2.5)
        ),

        (
            x + size * math.cos(theta - 2.5),
            y + size * math.sin(theta - 2.5)
        ),
    ]

    pygame.draw.polygon(screen, color, pts)


def wrap_angle(angle):

    return math.atan2(
        math.sin(angle),
        math.cos(angle)
    )


def unicycle_step(state, v, w):

    x, y, theta = state

    x += v * math.cos(theta) * DT
    y += v * math.sin(theta) * DT
    theta += w * DT

    return np.array([x, y, theta])


def leader_step_with_bounds(state, v, w):

    x, y, theta = state

    theta = theta + w * DT

    x_new = x + v * math.cos(theta) * DT
    y_new = y + v * math.sin(theta) * DT

    bounced = False

    # bounce on vertical walls
    if x_new <= 0 or x_new >= WIDTH:
        theta = math.pi - theta
        bounced = True

    # bounce on horizontal walls
    if y_new <= 0 or y_new >= HEIGHT:
        theta = -theta
        bounced = True

    # recompute after bounce
    if bounced:
        x_new = x + v * math.cos(theta) * DT
        y_new = y + v * math.sin(theta) * DT

    # keep inside bounds
    x_new = max(0, min(WIDTH, x_new))
    y_new = max(0, min(HEIGHT, y_new))

    return np.array([x_new, y_new, theta])


def add_noise(value, std):

    return value + random.gauss(0, std)


# cubic minimum-energy-like trajectory
def cubic_trajectory(p0, pf, T, t):

    a0 = p0
    a1 = 0

    a2 = 3 * (pf - p0) / (T**2)
    a3 = -2 * (pf - p0) / (T**3)

    pos = (
        a0
        + a1 * t
        + a2 * (t**2)
        + a3 * (t**3)
    )

    vel = (
        a1
        + 2 * a2 * t
        + 3 * a3 * (t**2)
    )

    return pos, vel


# ----------------------------
# MAIN LOOP
# ----------------------------
running = True

while running:

    screen.fill((25, 25, 25))

    # ----------------------------
    # EVENTS
    # ----------------------------
    for event in pygame.event.get():

        if event.type == pygame.QUIT:
            running = False

    # ----------------------------
    # LEADER UPDATE
    # ----------------------------
    leader = leader_step_with_bounds(
        leader,
        LEADER_V,
        LEADER_W
    )

    leader_history.append((time, leader.copy()))

    if len(leader_history) > MAX_TRAIL:
        leader_history.pop(0)

    # ----------------------------
    # SAMPLING
    # ----------------------------
    if time - last_sample_time >= SAMPLE_TIME:

        last_sample_time = time

        noisy_x = add_noise(leader[0], NOISE_STD)
        noisy_y = add_noise(leader[1], NOISE_STD)

        sampled_state = np.array([
            noisy_x,
            noisy_y,
            leader[2]
        ])

        sampled_targets.append((time, sampled_state))

        # one-step delayed tracking
        if len(sampled_targets) > 1:

            _, target_state = sampled_targets[-2]

            target = target_state
            target_start_time = time

    # ----------------------------
    # FOLLOWER CONTROL
    # ----------------------------
    if target is not None:

        T = SAMPLE_TIME

        t_local = time - target_start_time
        t_local = min(t_local, T)

        # smooth cubic trajectories
        x_traj, vx = cubic_trajectory(
            follower[0],
            target[0],
            T,
            t_local
        )

        y_traj, vy = cubic_trajectory(
            follower[1],
            target[1],
            T,
            t_local
        )

        # heading control
        desired_theta = math.atan2(vy, vx)

        theta = follower[2]

        v = math.sqrt(vx**2 + vy**2)

        angle_error = wrap_angle(
            desired_theta - theta
        )

        w = 3.0 * angle_error

        follower = unicycle_step(
            follower,
            v,
            w
        )

    follower_history.append((time, follower.copy()))

    if len(follower_history) > MAX_TRAIL:
        follower_history.pop(0)

    # ----------------------------
    # DRAW LEADER TRAJECTORY
    # ----------------------------
    for i in range(1, len(leader_history)):

        pygame.draw.line(
            screen,
            (0, 180, 0),
            leader_history[i - 1][1][:2],
            leader_history[i][1][:2],
            2
        )

    # ----------------------------
    # DRAW FOLLOWER TRAJECTORY
    # ----------------------------
    for i in range(1, len(follower_history)):

        pygame.draw.line(
            screen,
            (255, 140, 0),
            follower_history[i - 1][1][:2],
            follower_history[i][1][:2],
            2
        )

    # ----------------------------
    # DRAW SAMPLED POINTS
    # ----------------------------
    for _, s in sampled_targets:

        pygame.draw.circle(
            screen,
            (0, 120, 255),
            (int(s[0]), int(s[1])),
            4
        )

    # ----------------------------
    # DRAW ROBOTS
    # ----------------------------
    draw_robot(
        leader[0],
        leader[1],
        leader[2],
        (0, 255, 0)
    )

    draw_robot(
        follower[0],
        follower[1],
        follower[2],
        (255, 120, 0)
    )

    # ----------------------------
    # UPDATE DISPLAY
    # ----------------------------
    pygame.display.flip()

    # ----------------------------
    # SAVE GIF FRAMES
    # ----------------------------
    if EXPORT_GIF and int(time / DT) % 2 == 0:

        frame = pygame.surfarray.array3d(screen)

        frame = np.transpose(frame, (1, 0, 2))

        frames.append(Image.fromarray(frame))

    clock.tick(int(1 / DT))

    time += DT

# ----------------------------
# EXPORT GIF
# ----------------------------
if EXPORT_GIF and len(frames) > 0:

    output_path = "leader_follower.gif"

    frames[0].save(
        output_path,
        save_all=True,
        append_images=frames[1:],
        duration=int(DT * 1000),
        loop=0
    )

    print("\nGIF saved successfully!")
    print("Location:")
    print(os.path.abspath(output_path))

pygame.quit()
sys.exit()

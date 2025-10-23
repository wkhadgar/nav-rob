import random
import sys
from turtle import Vec2D

import PIL.Image as pil
import numpy as np
import sdl2
import sdl2.ext
import scipy as sci

import scripts.rrt as rrt

# --- Constants ---
MAP_WIDTH_METERS = 10
MAP_HEIGHT_METERS = 10

METER_TO_PIXELS = 100

WINDOW_WIDTH = MAP_WIDTH_METERS * METER_TO_PIXELS
WINDOW_HEIGHT = MAP_HEIGHT_METERS * METER_TO_PIXELS

# --- Colors (using sdl2.ext.Color for convenience) ---
WHITE = sdl2.ext.Color(255, 255, 255)
BLACK = sdl2.ext.Color(0, 0, 0)
RED = sdl2.ext.Color(255, 0, 0)
LOW_RED = sdl2.ext.Color(125, 0, 0)
GREEN = sdl2.ext.Color(0, 255, 0)
BLUE = sdl2.ext.Color(0, 0, 255)
YELLOW = sdl2.ext.Color(255, 255, 0)
CYAN = sdl2.ext.Color(0, 255, 255)
GREY = sdl2.ext.Color(128, 128, 128)


def draw_circle(renderer, center_x, center_y, radius, color):
    """
    Draws a circle on the renderer.
    This is a basic implementation since SDL_RenderDrawCircle is not in PySDL2.
    It uses the Midpoint circle algorithm.
    """
    x = radius - 1
    y = 0
    dx = 1
    dy = 1
    err = dx - (radius << 1)

    renderer.color = color

    while x >= y:
        renderer.draw_point([center_x + x, center_y + y])
        renderer.draw_point([center_x + y, center_y + x])
        renderer.draw_point([center_x - y, center_y + x])
        renderer.draw_point([center_x - x, center_y + y])
        renderer.draw_point([center_x - x, center_y - y])
        renderer.draw_point([center_x - y, center_y - x])
        renderer.draw_point([center_x + y, center_y - x])
        renderer.draw_point([center_x + x, center_y - y])

        if err <= 0:
            y += 1
            err += dy
            dy += 2

        if err > 0:
            x -= 1
            dx += 2
            err += dx - (radius << 1)


def draw_tree(renderer: sdl2.ext.Renderer, root: rrt.Node):
    for node in root.children:
        renderer.draw_line([root.vec[0], root.vec[1], node.vec[0], node.vec[1]], RED)
        draw_circle(renderer, node.vec[0], node.vec[1], 5, WHITE)
        draw_tree(renderer, node)


def calculate_collision_map(map_img_path: str, bot_radius=0.3) -> list[tuple[int, int]]:
    img = pil.open(map_img_path)
    img = img.resize((WINDOW_WIDTH, WINDOW_HEIGHT))
    img = img.convert("1")
    img_mat = np.array(img, dtype=np.uint8)
    img_mat -= 1

    diameter = 2 * int(bot_radius * np.sqrt(2) * METER_TO_PIXELS) + 1
    radius = diameter / 2
    y, x = np.indices((diameter, diameter))
    center = radius
    distance_squared = (x - center) ** 2 + (y - center) ** 2
    bot_mask = distance_squared <= radius ** 2
    bot_mask = bot_mask.astype(np.uint8)

    dilated_matrix = sci.ndimage.binary_dilation(img_mat, structure=bot_mask).astype(img_mat.dtype)
    wall_rows, wall_cols = np.where(dilated_matrix == 1)
    wall_points = np.vstack((wall_rows, wall_cols)).T

    return list(wall_points)


def draw_backtrace(renderer: sdl2.ext.Renderer, leaf: rrt.Node):
    if leaf.parent is not None:
        renderer.draw_line([leaf.vec[0], leaf.vec[1], leaf.parent.vec[0], leaf.parent.vec[1]], GREEN)
        draw_backtrace(renderer, leaf.parent)


def expand_tree(renderer, target_rrt: rrt.RRT, collision_map: list[tuple[int, int]], max_step=50, bias=0.1):
    if random.random() < (1 - bias):
        new_x = random.randint(0, WINDOW_WIDTH)
        new_y = random.randint(0, WINDOW_HEIGHT)
    else:
        new_x = target_rrt.goal.vec[0]
        new_y = target_rrt.goal.vec[1]
    q_rand = Vec2D(new_x, new_y)

    closest_node = target_rrt.find_closest_node(q_rand)

    to_new_vec = q_rand - closest_node.vec
    to_new_vec_len = abs(to_new_vec)
    if to_new_vec_len == 0:
        return
    to_new_vec *= (min(to_new_vec_len, max_step) / to_new_vec_len)

    q_new_pos = (closest_node.vec + to_new_vec)
    q_new_pos_discrete = int(q_new_pos[0]), int(q_new_pos[1])

    for px, py in collision_map:
        if px == q_new_pos_discrete[0] and py == q_new_pos_discrete[1]:
            draw_circle(renderer, q_new_pos_discrete[0], q_new_pos_discrete[1], 5, LOW_RED)
            return

    q_new = rrt.Node(*q_new_pos_discrete)
    closest_node.add_child(q_new)

    if q_new.vec == target_rrt.goal.vec:
        target_rrt.is_complete = True
        target_rrt.goal = q_new


def run():
    sdl2.ext.init()
    window = sdl2.ext.Window("SDL2 RRT Simulator", size=(WINDOW_WIDTH, WINDOW_HEIGHT))
    window.show()

    renderer = sdl2.ext.Renderer(window)

    rrt_ = rrt.RRT(500, 100, WINDOW_WIDTH - 10, WINDOW_HEIGHT - 10)

    wall = calculate_collision_map("/home/paulo/repos/nav-rob/src/world_simulation/worlds/map.png", bot_radius=0.35)
    renderer.draw_point(wall, GREY)

    draw_circle(renderer, rrt_.start.vec[0], rrt_.start.vec[1], 10, YELLOW)
    draw_circle(renderer, rrt_.goal.vec[0], rrt_.goal.vec[1], 10, CYAN)

    running = True
    while running:
        events = sdl2.ext.get_events()
        for event in events:
            if event.type == sdl2.SDL_QUIT:
                running = False
                break

        draw_tree(renderer, rrt_.root)
        if not rrt_.is_complete:
            expand_tree(renderer, rrt_, wall, max_step=int(0.25 * METER_TO_PIXELS), bias=0.08)
        else:
            draw_backtrace(renderer, rrt_.goal)

        renderer.present()
        sdl2.SDL_Delay(10)

    sdl2.ext.quit()
    return 0


if __name__ == "__main__":
    sys.exit(run())

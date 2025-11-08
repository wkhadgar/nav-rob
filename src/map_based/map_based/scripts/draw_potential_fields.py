#!/usr/bin/env python3

import os
import numpy as np
import PIL.Image as pil
import sdl2
import sdl2.ext
import scipy.ndimage as ndi

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

MAP_WIDTH_METERS = 10
MAP_HEIGHT_METERS = 10
METER_TO_PIXELS = 100
WINDOW_WIDTH = MAP_WIDTH_METERS * METER_TO_PIXELS
WINDOW_HEIGHT = MAP_HEIGHT_METERS * METER_TO_PIXELS

WHITE = sdl2.ext.Color(255, 255, 255)
BLACK = sdl2.ext.Color(0, 0, 0)
RED = sdl2.ext.Color(255, 0, 0)
GREEN = sdl2.ext.Color(0, 255, 0)
YELLOW = sdl2.ext.Color(255, 255, 0)
CYAN = sdl2.ext.Color(0, 0, 255)
GREY = sdl2.ext.Color(128, 128, 128)

def draw_circle(renderer, cx, cy, radius, color):
    x = radius - 1
    y = 0
    dx = 1
    dy = 1
    err = dx - (radius << 1)
    renderer.color = color
    while x >= y:
        renderer.draw_point([int(cx + x), int(cy + y)])
        renderer.draw_point([int(cx + y), int(cy + x)])
        renderer.draw_point([int(cx - y), int(cy + x)])
        renderer.draw_point([int(cx - x), int(cy + y)])
        renderer.draw_point([int(cx - x), int(cy - y)])
        renderer.draw_point([int(cx - y), int(cy - x)])
        renderer.draw_point([int(cx + y), int(cy - x)])
        renderer.draw_point([int(cx + x), int(cy - y)])
        if err <= 0:
            y += 1
            err += dy
            dy += 2
        if err > 0:
            x -= 1
            dx += 2
            err += dx - (radius << 1)

def draw_path(renderer: sdl2.ext.Renderer, path: list[tuple[int, int]], color):
    if not path:
        return
    renderer.color = color
    for i in range(len(path) - 1):
        p1 = path[i]
        p2 = path[i + 1]
        renderer.draw_line([p1[0], p1[1], p2[0], p2[1]])

def load_obstacle_map(map_img_path: str, bot_radius_meters: float) -> np.ndarray:
    img = pil.open(map_img_path)
    img = img.resize((WINDOW_WIDTH, WINDOW_HEIGHT))
    img = img.rotate(270)
    img = img.transpose(pil.FLIP_LEFT_RIGHT)
    img = img.convert("1")
    img_mat = (np.array(img, dtype=np.uint8))[::-1]
    img_mat -= 1

    diameter = 2 * int(bot_radius_meters * np.sqrt(2) * METER_TO_PIXELS) + 1
    radius = diameter / 2
    y, x = np.indices((diameter, diameter))
    center = radius
    distance_squared = (x - center) ** 2 + (y - center) ** 2
    bot_mask = distance_squared <= radius ** 2
    bot_mask = bot_mask.astype(np.uint8)

    dilated_matrix = ndi.binary_dilation(img_mat, structure=bot_mask).astype(img_mat.dtype)
    
    return dilated_matrix

def calc_attractive_potential(goal_px: tuple[int, int], map_shape: tuple[int, int], kp: float) -> np.ndarray:
    y, x = np.indices(map_shape)
    dist = np.hypot(x - goal_px[0], y - goal_px[1])
    
    U_att = 0.5 * kp * (dist ** 2)
    
    return U_att

def calc_repulsive_potential(obstacle_map: np.ndarray, rr_pixels: float, eta: float) -> np.ndarray:
    inv = 1 - obstacle_map
    dist_to_wall = ndi.distance_transform_edt(inv)
    
    rep = np.zeros(obstacle_map.shape, dtype=float)
    
    mask_near = dist_to_wall <= rr_pixels
    mask_on = dist_to_wall == 0
    calc = mask_near & ~mask_on
    
    d = dist_to_wall[calc] + 1e-6 
    rr = rr_pixels
    
    rep[calc] = 0.5 * eta * (1.0 / d - 1.0 / rr) ** 2
    rep[mask_on] = np.inf
    
    return rep

def get_motion_model() -> list[list[int]]:
    return [[1, 0], [0, 1], [-1, 0], [0, -1], [-1, -1], [-1, 1], [1, -1], [1, 1]]

class PotentialFieldPlanner:
    def __init__(self, obstacle_map: np.ndarray, goal_px: tuple[int, int],
                 rr_pixels_influence: float, start_px: tuple[int, int],
                 kp: float, eta: float):
        
        print("Calculando campo atrativo...")
        ug = calc_attractive_potential(goal_px, obstacle_map.shape, kp)
        
        print("Calculando campo repulsivo...")
        uo = calc_repulsive_potential(obstacle_map, rr_pixels_influence, eta)
        
        print("Somando campos...")
        self.pmap = ug + uo
        print("Cálculo dos campos concluído.")
        
        self.goal_px = goal_px
        self.motion = get_motion_model()
        self.path = [start_px]
        self.current_pos = start_px
        self.is_complete = False
        h, w = obstacle_map.shape
        self.map_width = w
        self.map_height = h

    def step(self):
        if self.is_complete:
            return

        ix, iy = self.current_pos
        min_p = self.pmap[iy, ix]
        min_ix, min_iy = ix, iy
        
        for dx, dy in self.motion:
            nx, ny = ix + dx, iy + dy
            if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                p = self.pmap[ny, nx]
                if p < min_p:
                    min_p = p
                    min_ix, min_iy = nx, ny
                    
        self.current_pos = (min_ix, min_iy)
        self.path.append(self.current_pos)
        
        dist_goal = np.hypot(self.current_pos[0] - self.goal_px[0], self.current_pos[1] - self.goal_px[1])
        
        if dist_goal < 2.0 or min_p == float('inf'):
            if min_p == float('inf'):
                print("Planejamento falhou: caminho bloqueado por obstáculo.")
            else:
                print(f"Planejamento concluído com {len(self.path)} pontos.")
            self.is_complete = True
            
            if (min_ix, min_iy) == (ix, iy):
                print("Planejamento parado: preso em mínimo local ou no objetivo.")

    def get_path(self) -> list[tuple[int, int]]:
        return self.path

    def get_current_pos(self) -> tuple[int, int]:
        return self.current_pos

class PotentialFieldSDLNode(Node):
    def __init__(self):
        super().__init__("potential_field_sdl")

        default_map_path = os.path.expanduser("~/ros2_jazzy/src/nav-rob/src/world_simulation/worlds/map.png")
        self.declare_parameter("map_path", default_map_path)
        self.declare_parameter("bot_radius_m", 0.2)
        self.declare_parameter("kp", 0.005) 
        self.declare_parameter("eta", 10000.0) 
        self.declare_parameter("influence_m", 0.5) 
        self.declare_parameter("start_px", [500, 100])
        self.declare_parameter("goal_px", [800,900])

        self.path_pub = self.create_publisher(String, "/path_points", 10)

        sdl2.ext.init()
        self.window = sdl2.ext.Window("SDL2 Potential Field Planner", size=(WINDOW_WIDTH, WINDOW_HEIGHT))
        self.window.show()
        self.renderer = sdl2.ext.Renderer(self.window)

        self.background_texture = sdl2.SDL_CreateTexture(
            self.renderer.sdlrenderer,
            sdl2.SDL_PIXELFORMAT_RGBA8888,
            sdl2.SDL_TEXTUREACCESS_TARGET,
            WINDOW_WIDTH, WINDOW_HEIGHT
        )
        self.get_logger().info("Textura de cache do mapa criada.")

        map_path = self.get_parameter("map_path").get_parameter_value().string_value
        bot_radius_m = self.get_parameter("bot_radius_m").get_parameter_value().double_value
        kp = self.get_parameter("kp").get_parameter_value().double_value
        eta = self.get_parameter("eta").get_parameter_value().double_value
        influence_m = self.get_parameter("influence_m").get_parameter_value().double_value

        start_px_param = self.get_parameter("start_px").get_parameter_value().integer_array_value
        goal_px_param = self.get_parameter("goal_px").get_parameter_value().integer_array_value
        self.start_px = (int(start_px_param[0]), int(start_px_param[1]))
        self.goal_px = (int(goal_px_param[0]), int(goal_px_param[1]))

        if not os.path.exists(map_path):
            self.get_logger().error(f"Mapa não encontrado: {map_path}")
            rclpy.shutdown()
            return

        self.get_logger().info(f"Carregando mapa: {map_path}")
        obstacle_map = load_obstacle_map(map_path, bot_radius_m)
        
        wy, wx = np.where(obstacle_map == 1)
        wall_points = list(zip(wx, wy)) 
        self.get_logger().info("Mapa carregado.")

        self.get_logger().info("Pré-renderizando o mapa estático na textura...")
        sdl2.SDL_SetRenderTarget(self.renderer.sdlrenderer, self.background_texture)
        
        self.renderer.color = BLACK
        self.renderer.clear()
        self.renderer.color = GREY
        if wall_points:
            self.renderer.draw_point(wall_points)
        draw_circle(self.renderer, self.start_px[0], self.start_px[1], 10, YELLOW)
        draw_circle(self.renderer, self.goal_px[0], self.goal_px[1], 10, CYAN)
        
        sdl2.SDL_SetRenderTarget(self.renderer.sdlrenderer, None)
        self.get_logger().info("Mapa pré-renderizado.")
        del wall_points
        
        rr_pixels_influence = influence_m * METER_TO_PIXELS
        
        self.get_logger().info("Iniciando cálculo dos campos potenciais...")
        self.planner = PotentialFieldPlanner(
            obstacle_map=obstacle_map,
            goal_px=self.goal_px,
            rr_pixels_influence=rr_pixels_influence,
            start_px=self.start_px,
            kp=kp,
            eta=eta
        )
        self.get_logger().info("Cálculo dos campos concluído. Iniciando simulação.")

        self.path_published = False
        self.timer = self.create_timer(0.01, self.run_step)

    def _publish_path_pixels(self, pts: list[tuple[int, int]]):
        if not pts:
            return
        
        path_msg = ""
        for point in pts:
            path_msg += f"{point[0] / METER_TO_PIXELS:.2f},{point[1] / METER_TO_PIXELS:.2f}\n"
        
        msg = String()
        msg.data = path_msg
        self.path_pub.publish(msg)
        self.get_logger().info(f"Caminho com {len(pts)} pontos publicado.")

    def run_step(self):
        for event in sdl2.ext.get_events():
            if event.type == sdl2.SDL_QUIT:
                if not self.path_published:
                    self._publish_path_pixels(self.planner.get_path())
                    self.path_published = True
                self.destroy_node()
                return

        sdl2.SDL_RenderCopy(self.renderer.sdlrenderer, self.background_texture, None, None)
        
        if not self.planner.is_complete:
            self.planner.step()
            cur = self.planner.get_current_pos()
            draw_circle(self.renderer, cur[0], cur[1], 3, RED)
        
        draw_path(self.renderer, self.planner.get_path(), GREEN)
        
        self.renderer.present()

    def destroy_node(self):
        if hasattr(self, "timer"):
            self.timer.cancel()
        
        if hasattr(self, "background_texture"):
            sdl2.SDL_DestroyTexture(self.background_texture)
            
        sdl2.ext.quit()
        self.get_logger().info("Nó encerrado.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PotentialFieldSDLNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node and rclpy.ok():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
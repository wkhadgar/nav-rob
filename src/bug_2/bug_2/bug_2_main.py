import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import String
import math
import argparse


def _make_point(x, y, z=0.0):
    """
    Cria um objeto geometry_msgs.msg.Point a partir das coordenadas fornecidas.

    Args:
        x (float): Coordenada X.
        y (float): Coordenada Y.
        z (float, optional): Coordenada Z. Padrão é 0.0.

    Returns:
        Point: Um objeto Point com as coordenadas especificadas.
    """
    p = Point()
    p.x = float(x)
    p.y = float(y)
    p.z = float(z)
    return p


class Bug2Controller(Node):
    MODE_TO_GOAL = 0
    MODE_WALL_FOLLOWING = 1

    def __init__(self, goal_x, goal_y):
        """
        Inicializa o nó Bug2Controller.

        Configura os subscribers para os dados de Lidar (/scan) e odometria (/odom),
        e os publishers para os comandos de velocidade (/cmd_vel) e marcadores de visualização.
        Define os parâmetros do controlador e inicia o loop de controle principal.

        Args:
            goal_x (float): A coordenada X do ponto de destino.
            goal_y (float): A coordenada Y do ponto de destino.
        """
        super().__init__('bug2_controller')

        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.goal_marker_publisher = self.create_publisher(Marker, '/goal_marker', 10)
        self.mline_marker_publisher = self.create_publisher(Marker, '/tangent_marker', 10)
        self.state_pub = self.create_publisher(String, '/bug_state', 10)
        self.last_state = None

        self.goal_x = goal_x
        self.goal_y = goal_y

        self.mode = self.MODE_TO_GOAL

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.yaw = 0.0

        self.dist_left = float('inf')
        self.dist_right = float('inf')
        self.dist_front = float('inf')
        self.dist_front_right = float('inf')

        self.hit_point = None

        self.distance_goal_tolerance = 0.3
        self.obstacle_detect_threshold = 0.5
        self.max_linear_speed = 0.4
        self.max_angular_speed = 1.0
        self.kp_ang = 1.5
        self.kp_lin = 0.5

        self.desired_wall_dist = 0.4
        self.k_dist = 1.2
        self.k_angle = 0.8

        self.start_x = None
        self.start_y = None
        self.start_point_set = False

        self.debug_counter = 0
        self.debug_log_every = 10

        self.timer = self.create_timer(0.1, self.control_loop)

    @staticmethod
    def _angle_to_index(angle, angle_min, angle_inc, ranges_len):
        """
        Converte um ângulo em radianos para o índice correspondente no array de leituras do Lidar.

        Args:
            angle (float): O ângulo em radianos a ser convertido.
            angle_min (float): O ângulo mínimo do scan (em radianos).
            angle_inc (float): O incremento angular entre as leituras (em radianos).
            ranges_len (int): O número total de leituras no scan.

        Returns:
            int: O índice correspondente ao ângulo fornecido.
        """
        rel = angle - angle_min
        idx = int(round(rel / angle_inc))
        return idx % ranges_len

    def _min_in_sector(self, ranges, angle_min, angle_inc, start_deg, end_deg):
        """
        Calcula a distância mínima válida dentro de um setor angular específico do scan do Lidar.

        Args:
            ranges (list): A lista de leituras de distância do Lidar.
            angle_min (float): O ângulo mínimo do scan.
            angle_inc (float): O incremento angular entre as leituras.
            start_deg (float): O ângulo inicial do setor (em graus).
            end_deg (float): O ângulo final do setor (em graus).

        Returns:
            float: A distância mínima encontrada no setor, ou infinito se nenhuma leitura válida for encontrada.
        """
        start_rad = math.radians(start_deg)
        end_rad = math.radians(end_deg)
        n = len(ranges)

        s_idx = self._angle_to_index(start_rad, angle_min, angle_inc, n)
        e_idx = self._angle_to_index(end_rad, angle_min, angle_inc, n)

        if s_idx <= e_idx:
            sector = ranges[s_idx:e_idx + 1]
        else:
            sector = list(ranges[s_idx:]) + list(ranges[:e_idx + 1])

        valid = [r for r in sector if (r and math.isfinite(r) and r > 0.0)]
        return min(valid) if valid else float('inf')

    def scan_callback(self, msg: LaserScan):
        """
        Callback para processar mensagens do Lidar (sensor_msgs/LaserScan).

        Atualiza as distâncias aos obstáculos nas direções frontal, esquerda, direita
        e frontal-direita, usando os dados do scan.

        Args:
            msg (LaserScan): A mensagem recebida do tópico /scan.
        """
        ranges = list(msg.ranges)
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment

        try:
            self.dist_left = self._min_in_sector(ranges, angle_min, angle_inc, 80, 100)
            self.dist_right = self._min_in_sector(ranges, angle_min, angle_inc, -100, -80)
            self.dist_front = self._min_in_sector(ranges, angle_min, angle_inc, -30, 30)
            self.dist_front_right = self._min_in_sector(ranges, angle_min, angle_inc, -100, -80)
        except Exception as e:
            self.get_logger().warn(f"Erro em scan_callback: {e}")
            self.dist_front = self.dist_front_right = float('inf')

    def odom_callback(self, msg: Odometry):
        """
        Callback para processar mensagens de odometria (nav_msgs/Odometry).

        Atualiza a posição (x, y) e a orientação (yaw) atuais do robô.
        Na primeira chamada, define o ponto de partida do robô.

        Args:
            msg (Odometry): A mensagem recebida do tópico /odom.
        """
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

        if not self.start_point_set:
            self.start_x = self.robot_x
            self.start_y = self.robot_y
            self.start_point_set = True
            self.get_logger().info(f"Ponto inicial: ({self.start_x:.2f}, {self.start_y:.2f})")

    def publish_goal_marker(self):
        """
        Publica um marcador de visualização (Marker) para representar o alvo no RViz.
        """
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "goal_namespace"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = float(self.goal_x)
        marker.pose.position.y = float(self.goal_y)
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.5
        marker.color.a = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.lifetime.sec = 5
        self.goal_marker_publisher.publish(marker)

    def publish_mline(self):
        """
        Publica um marcador de visualização (Marker) para representar a linha M (linha reta
        do ponto inicial ao alvo) no RViz.
        """
        if not self.start_point_set:
            return

        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "mline_namespace"
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.points = [
            _make_point(self.start_x, self.start_y),
            _make_point(self.goal_x, self.goal_y)
        ]

        marker.scale.x = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        self.mline_marker_publisher.publish(marker)

    def control_loop(self):
        """
        Loop de controle principal, executado periodicamente por um timer.

        Verifica a distância até o alvo e decide qual modo de navegação usar:
        `move_to_goal` ou `follow_wall`. Publica os comandos de velocidade resultantes.
        """
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = "base_link"

        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        distance_to_goal = math.hypot(dx, dy)

        self.publish_goal_marker()
        self.publish_mline()
        self.debug_counter += 1

        if distance_to_goal < self.distance_goal_tolerance:
            self.get_logger().info("Chegou no goal.")
            self.cmd_pub.publish(TwistStamped())
            self.publish_state()
            return

        if self.mode == self.MODE_TO_GOAL:
            self.move_to_goal(twist_stamped, distance_to_goal)
        elif self.mode == self.MODE_WALL_FOLLOWING:
            self.follow_wall(twist_stamped)

        twist_stamped.twist.linear.x = max(min(twist_stamped.twist.linear.x, self.max_linear_speed), -self.max_linear_speed)
        twist_stamped.twist.angular.z = max(min(twist_stamped.twist.angular.z, self.max_angular_speed), -self.max_angular_speed)

        self.cmd_pub.publish(twist_stamped)
        self.publish_state()

        if self.debug_counter >= self.debug_log_every:
            self.debug_counter = 0
            self._log_debug_state(twist_stamped, distance_to_goal)

    def move_to_goal(self, twist_stamped: TwistStamped, distance_to_goal: float):
        """
        Implementa a lógica de controle para mover o robô diretamente em direção ao alvo.

        Calcula o erro angular e aplica um controle proporcional para alinhar o robô
        com o alvo. Se um obstáculo for detectado à frente, muda para o modo `MODE_WALL_FOLLOWING`.

        Args:
            twist_stamped (TwistStamped): A mensagem de comando de velocidade a ser preenchida.
            distance_to_goal (float): A distância euclidiana atual até o alvo.
        """
        target_theta = math.atan2(self.goal_y - self.robot_y, self.goal_x - self.robot_x)
        angle_diff = math.atan2(math.sin(target_theta - self.yaw), math.cos(target_theta - self.yaw))

        if self.dist_front < self.obstacle_detect_threshold:
            self.get_logger().info("Mudando para WALL FOLLOWING")
            self.mode = self.MODE_WALL_FOLLOWING
            self.hit_point = (self.robot_x, self.robot_y)
            return

        twist_stamped.twist.angular.z = self.kp_ang * angle_diff
        twist_stamped.twist.linear.x = 0.0 if abs(angle_diff) > 0.7 else min(self.kp_lin * distance_to_goal, self.max_linear_speed)

    def follow_wall(self, twist_stamped: TwistStamped):
        """
        Implementa a lógica de controle para seguir uma parede (obstáculo).

        Usa as distâncias do Lidar para manter o robô se movendo ao longo de uma parede.
        Verifica continuamente se a linha M foi cruzada e se é vantajoso retornar
        ao modo `MODE_TO_GOAL`.

        Args:
            twist_stamped (TwistStamped): A mensagem de comando de velocidade a ser preenchida.
        """
        front_blocked = self.dist_front < self.obstacle_detect_threshold
        front_right_has_wall = self.dist_front_right < 0.6

        if front_blocked:
            twist_stamped.twist.linear.x = 0.0
            twist_stamped.twist.angular.z = 0.6
        elif front_right_has_wall and front_blocked:
            twist_stamped.twist.linear.x = 0.0
            twist_stamped.twist.angular.z = 0.6
        elif front_right_has_wall:
            twist_stamped.twist.linear.x = 0.15
            twist_stamped.twist.angular.z = 0.0
        else:
            twist_stamped.twist.linear.x = 0.5
            twist_stamped.twist.angular.z = -0.8

        if self.is_on_mline() and self.hit_point is not None:
            dist_now = math.hypot(self.robot_x - self.goal_x, self.robot_y - self.goal_y)
            dist_hit = math.hypot(self.hit_point[0] - self.goal_x, self.hit_point[1] - self.goal_y)
            if dist_now < dist_hit:
                self.mode = self.MODE_TO_GOAL

    def is_on_mline(self, threshold=0.2):
        """
        Verifica se a posição atual do robô está sobre a linha M (linha reta do início ao fim).

        Calcula a distância perpendicular do robô à linha M e a compara com um limiar.

        Args:
            threshold (float, optional): A distância máxima da linha M para considerar que o robô está sobre ela. Padrão é 0.2.

        Returns:
            bool: True se o robô estiver na linha M, False caso contrário.
        """
        if self.start_x is None or self.start_y is None:
            return False

        x1, y1 = self.start_x, self.start_y
        x2, y2 = self.goal_x, self.goal_y

        if x1 == x2 and y1 == y2:
            return True

        num = abs((y2 - y1) * self.robot_x - (x2 - x1) * self.robot_y + x2 * y1 - y2 * x1)
        den = math.hypot(y2 - y1, x2 - x1)
        dist = num / den if den > 1e-6 else float('inf')
        return dist < threshold

    def publish_state(self):
        """
        Publica o estado atual do algoritmo Bug2 ('TO_GOAL' ou 'WALL_FOLLOW') em um tópico.
        A publicação só ocorre se o estado tiver mudado desde a última vez.
        """
        mode_str = "TO_GOAL" if self.mode == self.MODE_TO_GOAL else "WALL_FOLLOW"
        if mode_str != self.last_state:
            msg = String()
            msg.data = mode_str
            self.state_pub.publish(msg)
            self.get_logger().info(f"Bug state → {mode_str}")
            self.last_state = mode_str

    def _log_debug_state(self, twist_stamped: TwistStamped, distance_to_goal: float):
        """
        Imprime uma mensagem de log detalhada com o estado atual do robô e do controlador.

        Args:
            twist_stamped (TwistStamped): A mensagem de comando de velocidade atual.
            distance_to_goal (float): A distância euclidiana atual até o alvo.
        """
        mode_str = "TO_GOAL" if self.mode == self.MODE_TO_GOAL else "WALL_FOLLOW"
        hit_str = f"{self.hit_point[0]:.3f},{self.hit_point[1]:.3f}" if self.hit_point else "None"
        debug_msg = (
            f"[{mode_str}] pos=({self.robot_x:.3f},{self.robot_y:.3f}) yaw={self.yaw:.3f} "
            f"goal=({self.goal_x:.3f},{self.goal_y:.3f}) d_goal={distance_to_goal:.3f} "
            f"cmd_lin={twist_stamped.twist.linear.x:.3f} cmd_ang={twist_stamped.twist.angular.z:.3f} hit={hit_str}"
        )
        self.get_logger().info(debug_msg)


def main(args=None):
    """
    Função principal que inicializa o nó ROS2 e o executa.

    Lê as coordenadas do alvo a partir dos argumentos da linha de comando,
    cria uma instância do Bug2Controller e o mantém em execução até que seja interrompido.
    """
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument("--goal_x", type=float, required=True)
    parser.add_argument("--goal_y", type=float, required=True)
    parsed_args, _ = parser.parse_known_args()

    node = Bug2Controller(parsed_args.goal_x, parsed_args.goal_y)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

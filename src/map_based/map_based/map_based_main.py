import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import String
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


class MapBasedController(Node):

    def __init__(self, start_x, start_y, goal_x, goal_y):
        super().__init__('map_based_controller')

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(String, '/path_points', self.get_path_callback, 10)

        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.goal_marker_publisher = self.create_publisher(Marker, '/goal_marker', 10)
        self.mline_marker_publisher = self.create_publisher(Marker, '/tangent_marker', 10)

        self.goal_x = goal_x
        self.goal_y = goal_y

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.yaw = 0.0

        self.start_x = start_x
        self.start_y = start_y

        self.max_linear_speed = 0.4
        self.max_angular_speed = 1.0
        self.kp_ang = 1.5

        self.path_to_follow = []
        self.path_index = 0

        self.path_target_tolerance = 0.05
        self.turn_in_place_threshold = np.pi / 4

        self.k_dist = 1.2
        self.k_angle = 0.8

        self.debug_log_every = 10
        self.debug_counter = 0

        self.publish_goal_marker()

    def get_path_callback(self, msg: String):
        tuples = str(msg.data)[:-1].strip().split("\n")
        tuples = [tup.split(",") for tup in tuples]
        points = [[-float(y) + 5, -float(x) + 5] for x, y in tuples]

        self.path_to_follow = points
        self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg: Odometry):
        """
        Callback para processar mensagens de odometria (nav_msgs/Odometry).

        Atualiza a posição (x, y) e a orientação (yaw) atuais do robô.
        Na primeira chamada, define o ponto de partida do robô.

        Args:
            msg (Odometry): A mensagem recebida do tópico /odom.
        """
        self.robot_x = self.start_x + msg.pose.pose.position.x
        self.robot_y = self.start_y + msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = np.arctan2(siny_cosp, cosy_cosp)

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

    def publish_path_marker(self, path_coordinates: list):
        """
        Publica um marcador de visualização (Marker) para representar um caminho
        (linha composta por múltiplos pontos) no RViz.

        :param path_coordinates: Uma lista de tuplas ou listas [(x1, y1), (x2, y2), ...]
        """

        if not path_coordinates or len(path_coordinates) < 2:
            return

        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "mline_namespace"
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.points = []
        for (x_coord, y_coord) in path_coordinates:
            marker.points.append(_make_point(-(self.start_x - x_coord), -(self.start_y - y_coord)))

        marker.scale.x = 0.05
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0

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
        distance_to_goal = np.hypot(dx, dy)

        self.publish_path_marker(self.path_to_follow)
        self.debug_counter += 1

        if distance_to_goal < 0.15:
            self.get_logger().info("Chegou no goal.")
            self.cmd_pub.publish(TwistStamped())
            return

        self.follow_path(twist_stamped)

        twist_stamped.twist.linear.x = max(min(twist_stamped.twist.linear.x, self.max_linear_speed),
                                           -self.max_linear_speed)
        twist_stamped.twist.angular.z = max(min(twist_stamped.twist.angular.z, self.max_angular_speed),
                                            -self.max_angular_speed)

        self.cmd_pub.publish(twist_stamped)

        if self.debug_counter >= self.debug_log_every:
            self._log_debug_state(twist_stamped, distance_to_goal)

    def follow_path(self, twist_stamped: TwistStamped):
        """
        Persegue um waypoint de cada vez da lista `self.path_to_follow`.
        Quando o robô chega perto de um waypoint, ele avança para o próximo
        até que o caminho esteja completo.

        Args:
            twist_stamped (TwistStamped): A mensagem de comando de velocidade a ser preenchida.
        """

        # Verifica se o caminho existe e se ainda temos pontos para seguir
        if not self.path_to_follow or self.path_index >= len(self.path_to_follow):
            if len(self.path_to_follow) == 0:
                self.get_logger().info("Caminho vazio.")

            twist_stamped.twist.linear.x = 0.0
            twist_stamped.twist.angular.z = 0.0
            self.path_to_follow = []
            self.path_index = 0
            return

        target_x, target_y = self.path_to_follow[self.path_index]

        # 2. Calcular a distância e o ângulo até o alvo
        dist_to_target = np.hypot(target_x - self.robot_x, target_y - self.robot_y)

        # 3. Verificar se chegamos ao waypoint atual
        if dist_to_target < self.path_target_tolerance:
            self.get_logger().info(f"Waypoint {self.path_index} alcançado.")
            self.path_index += 1  # Avança para o próximo waypoint

            # Se acabamos de avançar para *além* do último ponto, o caminho terminou.
            if self.path_index >= len(self.path_to_follow):
                self.get_logger().info("Caminho concluído! Retornando ao modo TO_GOAL.")
                twist_stamped.twist.linear.x = 0.0
                twist_stamped.twist.angular.z = 0.0
                self.path_to_follow = []  # Limpa o caminho
                self.path_index = 0
                return

            twist_stamped.twist.linear.x = 0.0
            twist_stamped.twist.angular.z = 0.0
            return

        desired_angle = np.arctan2(target_y - self.robot_y, target_x - self.robot_x)
        angle_error = desired_angle - self.yaw

        angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))

        angular_vel = self.kp_ang * angle_error

        if abs(angle_error) > self.turn_in_place_threshold:
            linear_vel = 0.0
        else:
            linear_vel = self.max_linear_speed

        twist_stamped.twist.linear.x = linear_vel
        twist_stamped.twist.angular.z = angular_vel

    def _log_debug_state(self, twist_stamped: TwistStamped, distance_to_goal: float):
        """
        Imprime uma mensagem de log detalhada com o estado atual do robô e do controlador.

        Args:
            twist_stamped (TwistStamped): A mensagem de comando de velocidade atual.
            distance_to_goal (float): A distância euclidiana atual até o alvo.
        """
        current_goal = self.path_to_follow[self.path_index]
        debug_msg = (
            f"[Waypoint {self.path_index}] pos=({self.robot_x:.3f},{self.robot_y:.3f}) yaw={self.yaw:.3f} "
            f"goal=({current_goal[0]:.3f},{current_goal[1]:.3f}) d_goal={distance_to_goal:.3f} "
            f"cmd_lin={twist_stamped.twist.linear.x:.3f} cmd_ang={twist_stamped.twist.angular.z:.3f}"
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
    parser.add_argument("--x_pose", type=float, required=True)
    parser.add_argument("--y_pose", type=float, required=True)
    parser.add_argument("--goal_x", type=float, required=True)
    parser.add_argument("--goal_y", type=float, required=True)
    parsed_args, _ = parser.parse_known_args()

    node = MapBasedController(parsed_args.x_pose, parsed_args.y_pose, parsed_args.goal_x, parsed_args.goal_y)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__("obstacle_avoidance")

        # Параметри топіків
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        
        # Ціль за перешкодами
        self.declare_parameter("goal_x", 2.2)
        self.declare_parameter("goal_y", 3.5)

        # --- НАЛАШТУВАННЯ СИЛ (Тюнінг для проходження перешкод) ---
        self.k_att = 0.5          # Сила тяги до цілі 
        self.k_rep = 0.5          # Сила відштовхування 
        self.k_tangent = 1.5      # Сила "обтікання" 
        self.dist_threshold = 0.8  # Реакція на стіни далі (80 см)

        self.goal_x = self.get_parameter("goal_x").value
        self.goal_y = self.get_parameter("goal_y").value
        
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.odom_received = False

        # Підписки та паблішер
        self.create_subscription(LaserScan, self.get_parameter("scan_topic").value, self.scan_callback, 10)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self.odom_callback, 10)
        self.pub_vel = self.create_publisher(TwistStamped, self.get_parameter("cmd_vel_topic").value, 10)

        self.get_logger().info(f"🚀 СТАРТ! Ціль: ({self.goal_x}, {self.goal_y})")

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.odom_received = True
        
        # Перетворення кватерніона в кут Yaw (куди дивиться робот)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg: LaserScan):
        if not self.odom_received:
            return

        # 1. Розрахунок дистанції до цілі
        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        dist_to_goal = math.sqrt(dx**2 + dy**2)

        if dist_to_goal > 0:
            f_att_x = self.k_att * (dx / dist_to_goal)
            f_att_y = self.k_att * (dy / dist_to_goal)
        else:
            f_att_x, f_att_y = 0.0, 0.0

        # 3. Сила відштовхування (Repulsive Force)
        f_rep_x = 0.0
        f_rep_y = 0.0
        rays_count = 0

        angle_min = msg.angle_min
        angle_inc = msg.angle_increment

        for i, dist in enumerate(msg.ranges):
            # Ігноруємо сміття та сам корпус
            if math.isinf(dist) or math.isnan(dist) or dist < 0.20:
                continue

            if dist < self.dist_threshold:
                obs_angle = angle_min + (i * angle_inc) + self.robot_yaw
                rep_mag = self.k_rep * (1.0/dist - 1.0/self.dist_threshold) / (dist**2)
                
                # ФІКС 1: Жорсткий ліміт на силу відштовхування
                rep_mag = min(rep_mag, 3.0) 

                f_rep_x -= rep_mag * math.cos(obs_angle)
                f_rep_y -= rep_mag * math.sin(obs_angle)
                rays_count += 1

        if rays_count > 0:
            f_rep_x /= rays_count
            f_rep_y /= rays_count
            
            # ФІКС 2: "Розумна" дотична сила (обтікання стіни)
            t_left_x, t_left_y = -f_rep_y, f_rep_x
            t_right_x, t_right_y = f_rep_y, -f_rep_x
            
            # Скалярний добуток покаже, який з напрямків ближчий до вектора цілі
            dot_left = t_left_x * f_att_x + t_left_y * f_att_y
            dot_right = t_right_x * f_att_x + t_right_y * f_att_y
            
            # Вибираємо найвигідніший шлях
            if dot_left > dot_right:
                tangent_x, tangent_y = t_left_x * self.k_tangent, t_left_y * self.k_tangent
            else:
                tangent_x, tangent_y = t_right_x * self.k_tangent, t_right_y * self.k_tangent
                
            f_rep_x += tangent_x
            f_rep_y += tangent_y

        # 4. Сумарний вектор руху
        total_x = f_att_x + f_rep_x
        total_y = f_att_y + f_rep_y

        # Визначаємо кут повороту
        target_yaw = math.atan2(total_y, total_x)
        angle_err = target_yaw - self.robot_yaw
        angle_err = math.atan2(math.sin(angle_err), math.cos(angle_err)) # Нормалізація від -pi до pi

        # 5. Створення команди руху
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        
        # ПЕРЕВІРКА ФІНІШУ
        if dist_to_goal < 0.15: 
            self.get_logger().info("🎯 Ціль досягнута! Зупинка.", throttle_duration_sec=2.0)
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0
        else:
            # ЛОГІКА РУХУ
            if abs(angle_err) > 1.0: 
                # Якщо треба сильно розвернутися (> 57 градусів), стоїмо на місці і крутимось
                cmd.twist.linear.x = 0.0
            else:
                # Їдемо вперед. Швидкість падає, якщо ми дивимось не зовсім туди
                # max() захищає від їзди задом
                forward_speed = min(0.2, 0.5 * dist_to_goal)
                cmd.twist.linear.x = forward_speed * max(0.0, math.cos(angle_err))
            
            # Повертаємо пропорційно помилці
            cmd.twist.angular.z = max(-1.2, min(1.2, 2.0 * angle_err))

        self.pub_vel.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Зупинка вузла...")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped  # Змінено з Twist

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__("obstacle_avoidance")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("goal_x", 3.0)
        self.declare_parameter("goal_y", 3.0)

        # Коефіцієнти (можна тюнити)
        self.k_att = 0.5      # Сила притягання до цілі
        self.k_rep = 0.1      # Сила відштовхування від стін
        self.dist_threshold = 1.2  # Радіус дії перешкод

        self.goal_x = self.get_parameter("goal_x").value
        self.goal_y = self.get_parameter("goal_y").value
        
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Підписки та паблішер (тепер TwistStamped)
        self.create_subscription(LaserScan, self.get_parameter("scan_topic").value, self.scan_callback, 10)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self.odom_callback, 10)
        self.pub_vel = self.create_publisher(TwistStamped, self.get_parameter("cmd_vel_topic").value, 10)

        self.get_logger().info(f"🚀 Obstacle Avoidance started. Goal: ({self.goal_x}, {self.goal_y})")

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Перетворення кватерніона в кут Yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg: LaserScan):
        # 1. Сила притягання (Attractive Force)
        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        dist_to_goal = math.sqrt(dx**2 + dy**2)

        f_att_x = self.k_att * dx
        f_att_y = self.k_att * dy

        # 2. Сила відштовхування (Repulsive Force)
        f_rep_x = 0.0
        f_rep_y = 0.0

        angle_min = msg.angle_min
        angle_inc = msg.angle_increment

        for i, dist in enumerate(msg.ranges):
            if math.isinf(dist) or math.isnan(dist) or dist < 0.12:
                continue

            if dist < self.dist_threshold:
                # Кут променя у світі
                angle = angle_min + (i * angle_inc) + self.robot_yaw
                
                # Потенційне поле
                rep_mag = self.k_rep * (1.0/dist - 1.0/self.dist_threshold) / (dist**2)
                f_rep_x -= rep_mag * math.cos(angle)
                f_rep_y -= rep_mag * math.sin(angle)

        # 3. Сумарний вектор руху
        total_x = f_att_x + f_rep_x
        total_y = f_att_y + f_rep_y

        # Визначаємо потрібний кут повороту
        target_yaw = math.atan2(total_y, total_x)
        angle_err = target_yaw - self.robot_yaw
        angle_err = math.atan2(math.sin(angle_err), math.cos(angle_err)) 

        # Створення повідомлення TwistStamped
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        
        if dist_to_goal < 0.2:
            self.get_logger().info("🎯 Goal Reached!")
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0
        else:
            # Обмеження швидкості для TurtleBot3 Burger
            cmd.twist.linear.x = min(0.2, 0.5 * dist_to_goal) 
            cmd.twist.angular.z = 1.5 * angle_err

        self.pub_vel.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
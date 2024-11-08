import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from math import sqrt

class QuadrotorNode(Node):
    def __init__(self, quadrotor_name):
        super().__init__(quadrotor_name)
        
        # 목표 위치 구독자 설정
        self.subscription = self.create_subscription(
            Point, f'/{quadrotor_name}/goal_position', self.goal_callback, 10
        )
        
        # 속도 퍼블리셔 설정
        self.velocity_publisher = self.create_publisher(Twist, f'/{quadrotor_name}/cmd_vel', 10)
        
        # 현재 위치 및 목표 위치 초기화
        self.current_position = Point()
        self.goal_position = None
        self.reached_goal = False

        # Odometry 데이터 구독 설정
        self.odom_subscription = self.create_subscription(
            Odometry, f'/{quadrotor_name}/odom', self.odom_callback, 10
        )

    def goal_callback(self, msg):
        self.goal_position = msg
        self.reached_goal = False
        self.get_logger().info(f'Goal position received: {msg.x}, {msg.y}, {msg.z}')
        self.move_to_goal()

    def move_to_goal(self):
        if not self.goal_position:
            return

        rate = self.create_rate(10)
        tolerance = 0.5  # 허용 오차
        min_speed = 0.1  # 최소 속도 설정
        max_speed = 1.0  # 최대 속도 설정

        while rclpy.ok() and not self.reached_goal:
            # 현재 위치와 목표 위치 사이의 거리 계산
            distance = sqrt(
                (self.goal_position.x - self.current_position.x) ** 2 +
                (self.goal_position.y - self.current_position.y) ** 2 +
                (self.goal_position.z - self.current_position.z) ** 2
            )

            if distance < tolerance:
                self.get_logger().info('Reached goal position. Initiating landing.')
                self.land()
                self.reached_goal = True
                break

            velocity_msg = Twist()
            # X, Y, Z 방향 속도를 최소 및 최대 속도 제한 내에서 설정
            velocity_msg.linear.x = max(min_speed, min(max_speed, 0.5 * (self.goal_position.x - self.current_position.x)))
            velocity_msg.linear.y = max(min_speed, min(max_speed, 0.5 * (self.goal_position.y - self.current_position.y)))
            velocity_msg.linear.z = max(min_speed, min(max_speed, 0.5 * (self.goal_position.z - self.current_position.z)))

            # 속도 명령 발행
            self.velocity_publisher.publish(velocity_msg)
            self.get_logger().info(
                f'Moving to X: {self.goal_position.x}, Y: {self.goal_position.y}, Z: {self.goal_position.z} | '
                f'Current distance to goal: {distance:.2f}'
            )

            # 루프 속도에 맞춰 대기
            rate.sleep()

    def odom_callback(self, msg):
        # Odometry 데이터를 통해 현재 위치 갱신
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y
        self.current_position.z = msg.pose.pose.position.z
        self.get_logger().info(f'Current position updated: {self.current_position.x}, {self.current_position.y}, {self.current_position.z}')

    def land(self):
        # 착륙 시 Z축 하강 속도를 지면과 가까워질수록 점진적으로 줄임
        rate = self.create_rate(10)  # 10Hz
        while self.current_position.z > 0.1:  # 지면에 가까워질 때까지 반복
            distance_to_ground = max(0.1, self.current_position.z)  # 지면과의 거리
            descent_speed = max(0.1, min(0.5, 0.2 * distance_to_ground))  # 하강 속도 제한
            
            velocity_msg = Twist()
            velocity_msg.linear.z = -descent_speed
            self.velocity_publisher.publish(velocity_msg)
            self.get_logger().info(f'Descending... Current altitude: {self.current_position.z:.2f}')

            rate.sleep()

        # 착륙 후 모든 속도를 0으로 설정하여 멈춤
        velocity_msg = Twist()
        self.velocity_publisher.publish(velocity_msg)
        self.get_logger().info("Landed successfully.")

def main(args=None):
    rclpy.init(args=args)
    quadrotor_name = 'quadrotor'
    quadrotor_node = QuadrotorNode(quadrotor_name)
    rclpy.spin(quadrotor_node)
    quadrotor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
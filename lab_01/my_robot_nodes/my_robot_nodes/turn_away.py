
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool

from lab_interfaces.msg import ObstacleInfo
from my_robot_interfaces.msg import RobotStatus
from my_robot_interfaces.srv import SetTurnSpeed


class TurnAway(Node):

    def __init__(self):
        self.last_obstacle_distance = 0.0
        super().__init__('turn_away')

        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('turn_speed', 0.5)
        self.declare_parameter('max_turn_speed', 1.5)

        self.enabled = False

        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        self.status_pub = self.create_publisher(
            RobotStatus, '/robot_status', 10
        )

        self.obstacle_sub = self.create_subscription(
            ObstacleInfo,
            '/obstacle',
            self.obstacle_callback,
            10
        )

        self.enable_service = self.create_service(
            SetBool,
            '/turn_away_enable',
            self.enable_callback
        )

        self.speed_service = self.create_service(
            SetTurnSpeed,
            '/set_turn_speed',
            self.speed_callback
        )

        self.get_logger().info(
            'Turn Away node started. Initially DISABLED.'
        )

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def enable_callback(self, request, response):

        self.enabled = request.data

        if not self.enabled:
            self.publish_stop()

        status = RobotStatus()
        status.state = 'stopped'
        status.obstacle_distance = self.last_obstacle_distance
        self.status_pub.publish(status)

        response.success = True
        response.message = (
            'Obstacle avoidance enabled'
            if self.enabled
            else 'Obstacle avoidance disabled'
        )

        self.get_logger().info(response.message)

        return response

    def speed_callback(self, request, response):

        max_speed = self.get_parameter(
            'max_turn_speed'
        ).value

        if request.turn_speed <= 0 or request.turn_speed > max_speed:

            response.success = False
            response.message = (
                f'Turn speed must be greater than 0 '
                f'and no more than {max_speed}'
            )

        else:

            self.set_parameters([
                rclpy.parameter.Parameter(
                    'turn_speed',
                    rclpy.Parameter.Type.DOUBLE,
                    float(request.turn_speed)
                )
            ])

            response.success = True
            response.message = (
                f'Turn speed updated to {request.turn_speed}'
            )

        return response

    def obstacle_callback(self, msg):
          
        self.last_obstacle_distance = msg.distance   

        if not self.enabled:
            return

        command = Twist()
        status = RobotStatus()

        forward_speed = self.get_parameter(
            'forward_speed'
        ).value

        turn_speed = self.get_parameter(
            'turn_speed'
        ).value

        status.obstacle_distance = msg.distance

        if msg.too_close:

            command.linear.x = 0.0

            if msg.angle > 0:
                command.angular.z = -turn_speed
            else:
                command.angular.z = turn_speed

            status.state = 'turning'

        else:

            command.linear.x = forward_speed
            command.angular.z = 0.0

            status.state = 'forward'

        self.cmd_pub.publish(command)
        self.status_pub.publish(status)


def main(args=None):

    rclpy.init(args=args)
    node = TurnAway()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


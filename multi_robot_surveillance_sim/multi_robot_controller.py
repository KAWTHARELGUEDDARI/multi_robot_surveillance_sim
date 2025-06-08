import rclpy # pyright: ignore[reportMissingImports]
from rclpy.node import Node # pyright: ignore[reportMissingImports]
from multi_robot_surveillance_sim.surveillance_grid import SurveillanceGrid # pyright: ignore[reportMissingImports]
from multi_robot_surveillance_sim.multi_robot import MultiRobot # pyright: ignore[reportMissingImports]
from multi_robot_surveillance_sim.pathfinding import multi_a_star # pyright: ignore[reportMissingImports]
from visualization_msgs.msg import Marker, MarkerArray # pyright: ignore[reportMissingImports]
from geometry_msgs.msg import Point # pyright: ignore[reportMissingImports]



class MultiRobotController(Node):
    def __init__(self):
        super().__init__('multi_robot_controller')
        self.grid = SurveillanceGrid(15, 10, obstacles={(5, 5), (6, 5), (7, 5)}, points_of_interest={(3, 4), (7, 8), (12, 2)})
        start_positions = [(0, 0), (0, 1), (0, 2)]
        goals = [(3, 4), (7, 8), (12, 2)]
        self.robots = MultiRobot(len(start_positions), start_positions)
        paths = multi_a_star(self.grid, start_positions, goals)
        self.robots.set_paths(paths)

        self.marker_pub = self.create_publisher(MarkerArray, 'surveillance_markers', 10)
        self.timer = self.create_timer(0.5, self.move_robots)

        self.publish_initial_markers()
        self.get_logger().info('Multi-Robot Controller started')

    def publish_initial_markers(self):
        marker_array = MarkerArray()
        # Publish obstacles
        for idx, (x, y) in enumerate(self.grid.obstacles):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "obstacles"
            marker.id = idx
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = 0.0
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.r = 0.5
            marker.color.g = 0.5
            marker.color.b = 0.5
            marker.color.a = 1.0
            marker_array.markers.append(marker)
        # Publish points of interest
        for idx, (x, y) in enumerate(self.grid.points_of_interest):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "points_of_interest"
            marker.id = idx
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = 0.0
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def publish_paths(self):
        marker_array = MarkerArray()
        for idx, path in enumerate(self.robots.paths):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = f"path_robot_{idx}"
            marker.id = idx
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.1
            marker.color.r = 1.0 if idx == 0 else 0.0
            marker.color.g = 1.0 if idx == 1 else 0.0
            marker.color.b = 1.0 if idx == 2 else 0.0
            marker.color.a = 1.0
            for x, y in path:
                point = Point()
                point.x = float(x)
                point.y = float(y)
                point.z = 0.0
                marker.points.append(point)
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def publish_robot_positions(self):
        marker_array = MarkerArray()
        for idx, robot in enumerate(self.robots.robots):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = f"robot_{idx}"
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(robot.position[0])
            marker.pose.position.y = float(robot.position[1])
            marker.pose.position.z = 0.0
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.r = 1.0 if idx == 0 else 0.0
            marker.color.g = 1.0 if idx == 1 else 0.0
            marker.color.b = 1.0 if idx == 2 else 0.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def move_robots(self):
        all_done = all(not robot.path for robot in self.robots.robots)
        if all_done:
            self.get_logger().info('All robots reached their goals')
            self.timer.cancel()
            self.publish_robot_positions()
            return
        self.robots.move_step()
        self.publish_paths()
        self.publish_robot_positions()
        self.get_logger().info(f'Robots at {self.robots.positions}')

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly')
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
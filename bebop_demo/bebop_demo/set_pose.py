import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from tf_transformations import quaternion_from_euler
import json

class MultiRobotPosePublisher(Node):
    def __init__(self):
        super().__init__('multi_robot_pose_publisher')
        
        # Initialize robot names and ICs
        self.declare_parameter('robot_names', '[]')  # Empty list by default
        self.declare_parameter('initial_conditions', '[]')  # Empty list by default
        
        # Get and parse parameters
        try:
            self.robot_names = json.loads(self.get_parameter('robot_names').value)
            self.initial_conditions = json.loads(self.get_parameter('initial_conditions').value)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"ERROR de JSON: {str(e)}")
            raise
        
        # Make sure lengths match
        if len(self.robot_names) != len(self.initial_conditions):
            self.get_logger().error(
                f"Number of robots ({len(self.robot_names)}) "
                f"does not match number of initial conditions ({len(self.initial_conditions)})"
            )
            raise ValueError("Parameter lengths do not match.")
        
        # Dynamic publisher creation (one per robot)
        self._publishers = [
            self.create_publisher(Pose, f'{name}/set_pose', 10)
            for name in self.robot_names
        ]
        
        # Timer to publish initial poses once
        self.timer = self.create_timer(2.0, self.publish_initial_poses)

    def publish_initial_poses(self):
        """Publishes initial poses for all drones."""
        self.hd = 0.05075  # Takeoff height (adjustable)
        
        for i, (name, conditions) in enumerate(zip(self.robot_names, self.initial_conditions)):
            if len(conditions) < 4:
                self.get_logger().error(f"Invalid initial conditions for {name}: {conditions}")
                continue
            
            pose = Pose()
            pose.position.x = conditions[0]
            pose.position.y = conditions[1]
            pose.position.z = conditions[2] + self.hd  # Add takeoff altitude
            
            # Convert yaw (in radians) to a quaternion 
            q = quaternion_from_euler(0, 0, conditions[3])
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]
            
            self._publishers[i].publish(pose)
            self.get_logger().info(f"Published initial pose for {name}: {conditions}")
        
        self.timer.cancel()  # Stop after publishing

def main(args=None):
    rclpy.init(args=args)
    try:
        node = MultiRobotPosePublisher()
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"ERROR: {str(e)}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
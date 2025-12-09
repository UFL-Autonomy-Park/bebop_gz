#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist, Pose
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Bool, Int32
from std_srvs.srv import Empty
from .pid import PID

class Controller(Node):
    class State:
        IDLE = 0
        AUTOMATIC = 1
        TAKING_OFF = 2
        LANDING = 3
        EMERGENCY_STOP = 4

    def __init__(self):
        super().__init__('controller')
        
        # Parameters
        self.declare_parameter('frequency', 50.0)
        self.declare_parameter('robot_name', 'bebop3')
        self.declare_parameter('goal_name', 'goal')
        self.declare_parameter('takeoff_threshold', 0.04)
        self.declare_parameter('landing_threshold', 0.08)
        self.declare_parameter('takeoff_height', 1.0)  # New takeoff altitude 
        
        self.frequency = self.get_parameter('frequency').value
        self.robot_name = self.get_parameter('robot_name').value.strip()
        self.goal_name = self.get_parameter("goal_name").value.strip()
        self.takeoff_threshold = self.get_parameter('takeoff_threshold').value
        self.landing_threshold = self.get_parameter('landing_threshold').value
        self.takeoff_height = self.get_parameter('takeoff_height').value

        if not self.robot_name:
            self.get_logger().warn('The parameter "robot_name" is empty. "bebop104" will be used by default.')
            self.robot_name = 'bebop104'

        self.get_logger().info(f"Robot Name: {self.robot_name}")

        # Publishers
        qos = QoSProfile(depth=10)
        self.cmd_pub = self.create_publisher(Twist, f"/{self.robot_name}/cmd_vel", qos)
        self.cmd_des = self.create_publisher(Pose, f"/{self.robot_name}/setpointG", qos)
        self.cmd_enable = self.create_publisher(Bool, f"/{self.robot_name}/enable", qos)
        
        # Subscribers
        self.goal_sub = self.create_subscription(Pose, f"/{self.goal_name}", self.goal_changed, qos)
        self.pos_sub = self.create_subscription(Pose, f"/{self.robot_name}/pose", self.pos_changed, qos)
        self.state_sub = self.create_subscription(Int32, "/state", self.state_changed, qos)
        
        # Services
        self.takeoff_srv = self.create_service(Empty, 'takeoff', self.takeoff_callback)
        self.land_srv = self.create_service(Empty, 'land', self.land_callback)
        
        # PIDs
        self.pid_x = self.create_pid('X')
        self.pid_y = self.create_pid('Y')
        self.pid_z = self.create_pid('Z')
        self.pid_yaw = self.create_pid('Yaw')
        
        # State
        self.state = self.State.IDLE
        self.goal = Pose()
        self.current_pose = Pose()
        self.enable = False
        self.takeoff_complete = False 
        
        # Timer
        self.timer = self.create_timer(1.0 / self.frequency, self.control_loop)

    def create_pid(self, axis):
        prefix = f'PIDs.{axis}.'
        kp = self.get_param_or(prefix + 'kp', 0.0)
        kd = self.get_param_or(prefix + 'kd', 0.0)
        ki = self.get_param_or(prefix + 'ki', 0.0)
        min_output = self.get_param_or(prefix + 'minOutput', -1.0)
        max_output = self.get_param_or(prefix + 'maxOutput', 1.0)
        integrator_min = self.get_param_or(prefix + 'integratorMin', -0.5)
        integrator_max = self.get_param_or(prefix + 'integratorMax', 0.5)
        dt = 1.0 / self.frequency
        
        return PID(self, kp, kd, ki, min_output, max_output, integrator_min, integrator_max, dt, axis.lower())

    def get_param_or(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).get_parameter_value().double_value

    def state_changed(self, msg):
        """Handles state changes from the /state topic"""
        new_state = msg.data
        
        if new_state == self.State.IDLE:
            if self.state != self.State.IDLE:
                self.get_logger().info("Changing to IDLE state")
                self.state = self.State.IDLE
                self.enable = False
                
        elif new_state == self.State.AUTOMATIC:
            if self.state == self.State.TAKING_OFF and self.takeoff_complete:
                self.get_logger().info("Changing to AUTOMATIC state")
                self.state = self.State.AUTOMATIC
                self.enable = True
            elif self.state != self.State.TAKING_OFF:
                self.get_logger().warn("Cannot switch to AUTOMATIC: takeoff not complete")
                
        elif new_state == self.State.TAKING_OFF:
            if self.state != self.State.TAKING_OFF:
                self.get_logger().info("Changing to TAKING_OFF state")
                self.state = self.State.TAKING_OFF
                self.takeoff_complete = False
                self.enable = True
                
        elif new_state == self.State.LANDING:
            if self.state != self.State.LANDING:
                self.get_logger().info("Changing to LANDING state")
                self.state = self.State.LANDING
                self.enable = True
                
        elif new_state == self.State.EMERGENCY_STOP:
            if self.state != self.State.EMERGENCY_STOP:
                self.get_logger().warn("EMERGENCY STOP ACTIVATED")
                self.state = self.State.EMERGENCY_STOP
                self.enable = False
        
        # Publicar estado de enable
        self.cmd_enable.publish(Bool(data=self.enable))

    def goal_changed(self, msg):
        self.goal = msg

    def pos_changed(self, msg):
        self.current_pose = msg

    def takeoff_callback(self, request, response):
        self.get_logger().info('Takeoff requested!')
        if self.state != self.State.TAKING_OFF:
            self.state = self.State.TAKING_OFF
            self.takeoff_complete = False
            self.enable = True
            self.cmd_enable.publish(Bool(data=self.enable))
        return response

    def land_callback(self, request, response):
        self.get_logger().info('Landing requested!')
        if self.state != self.State.LANDING:
            self.state = self.State.LANDING
            self.enable = True
            self.cmd_enable.publish(Bool(data=self.enable))
        return response

    def control_loop(self):
        if self.state == self.State.TAKING_OFF:
            current_z = self.current_pose.position.z
            
            if not self.takeoff_complete:
                if current_z < self.takeoff_height - 0.05:  
                    msg = Twist()
                    msg.linear.z = float(self.pid_z.update(current_z, self.takeoff_height))
                    self.cmd_pub.publish(msg)
                else:
                    # Height reached
                    self.get_logger().info(f"Takeoff altitutde reached: {current_z:.2f}m")
                    self.takeoff_complete = True
            else:
                # Altitude control with PID
                msg = Twist()
                msg.linear.z = float(self.pid_z.update(current_z, self.takeoff_height))
                self.cmd_pub.publish(msg)

        elif self.state == self.State.LANDING:
            current_z = self.current_pose.position.z
            msg = Twist()
            
            if current_z > self.landing_threshold:
                # Controlled descent
                msg.linear.z = float(self.pid_z.update(current_z, 0.0)) * 0.5  
                self.cmd_pub.publish(msg)
            else:
                # Landing success
                self.get_logger().info("Landing success.")
                self.state = self.State.IDLE
                self.enable = False
                self.cmd_enable.publish(Bool(data=self.enable))
                self.cmd_pub.publish(Twist())

        elif self.state == self.State.AUTOMATIC and self.enable:
            try:
                # Get current and desired pose
                euler_c = euler_from_quaternion([
                    self.current_pose.orientation.x,
                    self.current_pose.orientation.y,
                    self.current_pose.orientation.z,
                    self.current_pose.orientation.w
                ])

                euler_d = euler_from_quaternion([
                    self.goal.orientation.x,
                    self.goal.orientation.y,
                    self.goal.orientation.z,
                    self.goal.orientation.w
                ])

                # Desired yaw and position
                x_d, y_d, z_d, yaw_d = self.goal.position.x, self.goal.position.y, self.goal.position.z, euler_d[2]

                # Control PID
                msg = Twist()
                msg.linear.x = float(self.pid_x.update(self.current_pose.position.x, x_d))
                msg.linear.y = float(self.pid_y.update(self.current_pose.position.y, y_d))
                msg.linear.z = float(self.pid_z.update(self.current_pose.position.z, z_d))
                msg.angular.z = float(self.pid_yaw.update(euler_c[2], yaw_d))
                self.cmd_pub.publish(msg)

                # Publish setpoint for visualization
                msg_goal = Pose()
                msg_goal.position = self.goal.position
                quat = quaternion_from_euler(0, 0, yaw_d)
                msg_goal.orientation.x = quat[0]
                msg_goal.orientation.y = quat[1]
                msg_goal.orientation.z = quat[2]
                msg_goal.orientation.w = quat[3]
                self.cmd_des.publish(msg_goal)

            except Exception as e:
                self.get_logger().error(f"Error in controller: {str(e)}")
                self.enable = False
                self.cmd_enable.publish(Bool(data=self.enable))

        elif self.state in [self.State.IDLE, self.State.EMERGENCY_STOP]:
            self.cmd_pub.publish(Twist())
            self.enable = False
            self.cmd_enable.publish(Bool(data=self.enable))

def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
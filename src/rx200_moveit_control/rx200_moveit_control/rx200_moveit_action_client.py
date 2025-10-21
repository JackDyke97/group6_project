#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Quaternion



class MoveItEEClient(Node):
    def __init__(self):
        super().__init__('rx200_moveit_control')

        self._client = ActionClient(self, MoveGroup, '/move_action')
        while not self._client.wait_for_server(1.0):
            self.get_logger().warning('Waiting for Action Server...')
        
        self.group_name = 'interbotix_arm'
        self.ee_link = 'rx200/ee_gripper_link'      #EE = End Effector
        self.base_link = 'rx200/base_link'

        self.get_logger().info('Node initialized successfully!')


    def send_pose(self, x ,y, z, w =1.0):
        pose = PoseStamped()
        pose.header.frame_id = self.base_link
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=w)

        req = MotionPlanRequest()
        req.group_name = self.group_name
        req.allowed_planning_time = 5.0
        req.num_planning_attempts = 3

        pc = PositionConstraint()
        pc.header.frame_id = self.base_link
        pc.link_name = self.ee_link
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE #set EE shape
        sp.dimensions = [0.01]
        pc.constraint_region.primitives = [sp]
        pc.constraint_region.primitive_poses = [pose.pose] #constraints position

        oc = OrientationConstraint() #constraints orientation
        oc.header.frame_id = self.base_link
        oc.link_name = self.ee_link
        oc.orientation = pose.pose.orientation
        oc.absolute_x_axis_tolerance = 0.05     #radians
        oc.absolute_y_axis_tolerance = 0.05
        oc.absolute_z_axis_tolerance = 0.05
        oc.weight = 1.0     #normalized weight


        goal_constraints = Constraints()
        goal_constraints.position_constraints = [pc]
        goal_constraints.orientation_constraints = [oc]
        req.goal_constraints = [goal]

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.look_around = False

        send_future = self._client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('MoveIt Goal rejected')
            return
        self.get_logger().info('MoveIt Goal Accepted')
        goal_handle.get_result_async().add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        state = getattr(feedback_msg.feedback, "state", "<unknown>")
        self.get_logger().info(f' [Feedback] {state}')

    def _result_cb(self, future):
        result = future.result().result
        code = getattr(result.error_code, 'val', 'unknown')
        self.get_logger().info(f'[RESULT] error_code {code} ')




def main():
    rclpy.init()
    node = MoveItEEClient()
    node.send_pose(0.25, 0.0, 0.15)  # single EE pose
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

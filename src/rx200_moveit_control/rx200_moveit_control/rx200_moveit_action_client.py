#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, JointConstraint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Quaternion
import time
import ast
from ast import literal_eval


class MoveItEEClient(Node):
    def __init__(self):
        super().__init__('rx200_moveit_control')

        self._client = ActionClient(self, MoveGroup, '/move_action')
        while not self._client.wait_for_server(1.0):
            self.get_logger().warning('Waiting for Action Server...')
        
        self.group_name_arm = 'interbotix_arm'
        self.ee_link = 'rx200/ee_gripper_link'      #EE = End Effector
        self.base_link = 'rx200/base_link'
        self.gripper_joint = 'left_finger'
        self.group_name_gripper = 'interbotix_gripper'

        self.declare_parameter('start_state_gripper', value = True)
        self.declare_parameter('waypoints', '[]')

        # Wait for the initial gripper move to complete
        self.send_gr_pose(self.get_parameter('start_state_gripper').value)
        self.wait_for_motion(timeout=5.0)

        self.get_logger().info('Node initialized successfully!')

        self.motion_done = False
        self.motion_result = None

    def send_pose(self, x ,y, z, w =1.0):
        self.motion_done = False
        self.motion_result = None

        pose = PoseStamped()
        pose.header.frame_id = self.base_link
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        req = MotionPlanRequest()
        req.group_name = self.group_name_arm
        req.allowed_planning_time = 10.0
        req.num_planning_attempts = 5
        req.start_state.is_diff = True #starts arm from its current state


        pc = PositionConstraint()
        pc.header.frame_id = self.base_link
        pc.link_name = self.ee_link
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE #set EE shape
        sp.dimensions = [0.05]
        pc.constraint_region.primitives = [sp]
        pc.constraint_region.primitive_poses = [pose.pose] #constraints position
        pc.weight = 1.0

        oc = OrientationConstraint() #constraints orientation
        oc.header.frame_id = self.base_link
        oc.link_name = self.ee_link
        oc.orientation = pose.pose.orientation
        oc.absolute_x_axis_tolerance = 1.0     #radians
        oc.absolute_y_axis_tolerance = 1.0
        oc.absolute_z_axis_tolerance = 1.0
        oc.weight = 1.0     #normalized weight

        goal_constraints = Constraints()
        goal_constraints.position_constraints = [pc]
        goal_constraints.orientation_constraints = [oc]
        req.goal_constraints = [goal_constraints]

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.look_around = False

        send_future = self._client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        send_future.add_done_callback(self._goal_response_cb)

    def cartesian_path(self, start_xyz, end_xyz, steps=10):
            x0, y0, z0 = start_xyz
            x1, y1, z1 = end_xyz

            for i in range(steps + 1):
                t = i / steps
                xi = x0 + (x1 - x0) * t
                yi = y0 + (y1 - y0) * t
                zi = z0 + (z1 - z0) * t

                self.get_logger().info(f"Moving step {i}/{steps}: ({xi:.3f}, {yi:.3f}, {zi:.3f})")
                self.send_pose(xi, yi, zi)
                success = self.wait_for_motion(timeout=10.0)
                if not success:
                    self.get_logger().error(f'step {i} failed. stopping movement')
                    return False
            return True
   
    def send_gr_pose(self,open = True):
        #move the gripper
        self.motion_done = False
        self.motion_result = None

        #create a simple goal for the gripper
        req = MotionPlanRequest()
        req.group_name = self.group_name_gripper
        req.allowed_planning_time = 2.0
        req.num_planning_attempts = 1

        jc = JointConstraint()
        jc.joint_name = self.gripper_joint
        jc.position = 0.035 if open else 0.0
        jc.tolerance_above = 0.01
        jc.tolerance_below = 0.01
        jc.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.joint_constraints = [jc]
        req.goal_constraints = [goal_constraints]
        
        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False

        send_future = self._client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_cb)


    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('MoveIt Goal rejected')
            self.motion_done = True
            self.motion_result = -1
            return
        self.get_logger().info('MoveIt Goal Accepted')
        goal_handle.get_result_async().add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        state = getattr(feedback_msg.feedback, "state", "<unknown>")
        self.get_logger().info(f' [Feedback] {state}')

    def _result_cb(self, future):
        result = future.result().result
        code = getattr(result.error_code, 'val', 'unknown')
        self.motion_result = code
        self.motion_done = True
        self.get_logger().info(f'[RESULT] error_code {code} ')

    #delay so that the arm completes its current movement or times out
    def wait_for_motion(self, timeout=10.0):
        start_time = time.time()
        while rclpy.ok() and not self.motion_done:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                self.get_logger().warning('Timeout waiting for motion result.')
                return False
        return self.motion_result == 1


# def directions_from_args():
#     args = sys.argv[1:]
#     waypoints = []
#     for arg in args:
#         try:
#             x, y, z = map(float, arg.split(','))
#             waypoints.append((x, y, z))
#         except ValueError:
#             print(f'[Error] Invalid argument: {arg}, skipping.')
#     return waypoints


def main():
    rclpy.init()
    node = MoveItEEClient()


#     #sets the target waypoints
#     waypoints = [
#     (0.35, 0.0, 0.25),   # Above the pickup location
#     (0.32, 0.0, 0.22),   # Move down to grasp
#     (0.30, 0.0, 0.20),   # Lift up
#     (0.15, 0.05, 0.20),  # Move diagonally above target
#     (0.10, 0.10, 0.20),  # Above place location
#     (0.10, 0.10, 0.10),  # Move down to place
#     (0.10, 0.10, 0.20)   # Lift up again
# ]

    waypoints_param = node.get_parameter('waypoints').get_parameter_value().string_value
    try:
        waypoints = ast.literal_eval(waypoints_param)
    except Exception as e:
        node.get_logger().error(f"Failed to parse waypoints: {e}")
        waypoints = []

    if not waypoints:
        node.get_logger().info("No waypoints provided. Exiting.")
        rclpy.shutdown()
        return

    

    for i in range(len(waypoints) - 1):
        start = waypoints[i]
        end = waypoints[i + 1]

        success = node.cartesian_path(start, end, steps=7)
        if not success:
            break

        # Gripper control
        if i == 0:  # close gripper after first segment
            node.send_gr_pose(False)
            node.wait_for_motion(timeout=5.0)
        elif i == len(waypoints) - 2:  # open gripper at last segment
            node.send_gr_pose(True)
            node.wait_for_motion(timeout=5.0)

        time.sleep(0.5) 

    # for step_number, (x, y, z) in enumerate(waypoints):
    #     node.get_logger().info(f'[{step_number+1}/{len(waypoints)}] Moving to: ({x}. {y}, {z})')
    #     # Send the pose goal
    #     node.send_pose(x, y, z)
    #     #wait for the result
    #     success = node.wait_for_motion(timeout=15.0)

    #     #check for failure
    #     if not success:
    #         node.get_logger().error(f'Move {step_number+1} failed (error code: {node.motion_result}). Stopping Seqeunce')
    #         break

    #     #gripper control
    #           #open gripper on second movement
    #     if step_number == 1: #0 indexed
    #         node.send_gr_pose(open=False)
    #         node.wait_for_motion(timeout=5.0) #waits until gripper finishes
    #     elif step_number == len(waypoints)-1: #open/release on last movement
    #         node.send_gr_pose(open=True)
    #         node.wait_for_motion(timeout=5.0)

    #     time.sleep(1.0) #small pause between movements

        node.get_logger().info('movement complete')

   # node.send_pose(0.25, 0.0, 0.15, w=1.0)  # single EE pose
    node.send_gr_pose(True)
    node.wait_for_motion(timeout=5.0)
    #rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

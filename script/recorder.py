#!/usr/bin/env python
import sys
import copy
import numpy as np
import rospy
import rosparam
import moveit_commander
import geometry_msgs
import visualization_msgs
import std_msgs
import moveit_msgs
from tf.transformations import quaternion_about_axis, quaternion_multiply
from moveit_commander.conversions import pose_to_list

# Ref: move_group_python_interface_tutorial.py
# https://github.com/ros-planning/moveit_tutorials/blob/master/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class MoveGroupPythonInteface(object):

    """MoveGroupPythonInteface"""

    def __init__(self, _group_name, _ee_name):
        super(MoveGroupPythonInteface, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)

        # group_name = rosparam.get_param("%s/group_name" % rospy.get_name())
        group_name = _group_name

        # Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        # kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        # Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        # for getting, setting, and updating the robot's internal understanding of the surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        # Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        # to a planning group (group of joints).
        # This interface can be used to plan and execute motions:
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group.set_end_effector_link(_ee_name)

        # Create a `DisplayTrajectory`_ ROS publisher which is used to display
        # trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        # Get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        # Get the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        # Get a list of all the groups in the robot:
        group_names = robot.get_group_names()

        print "============ INFO ============"
        rospy.logwarn("group_name: %s" % group_name)
        print "Available Planning Groups: ", robot.get_group_names()
        print "Planning frame: %s" % planning_frame
        print "End effector link: %s" % eef_link
        print "Robot joint space:"
        print robot.get_current_state().joint_state
        print "=============================="
        print ""

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.joint_names = robot.get_current_state().joint_state.name
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    # def force_set_joint_values(self, joint_values):
    #     idx = 0
    #     for joint_value in joint_values:
    #         name = self.joint_names[idx]
    #         self.robot.get_joint(name).move(joint_value)
    #         idx += 1
    #     self.move_group.stop()

    #     current_joints = self.move_group.get_current_joint_values()
    #     return all_close(joint_values, current_joints, 0.01)

    def go_to_joint_state(self, joint_goal):
        move_group = self.move_group

        # The Panda's zero configuration is at a
        # `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`

        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        # tip
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self, position, quat):
        """
        position: geometry_msgs/Point
        quat: geometry_msgs/Quaternion
        """
        move_group = self.move_group

        # end-effector:
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation = quat
        pose_goal.position = position

        move_group.set_pose_target(pose_goal)

        # Now, we call the planner to compute the plan and execute it.
        # True, False
        plan = move_group.go(wait=True)

        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        move_group.clear_pose_targets()

        # tip
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def get_current_joints(self):
        return self.move_group.get_current_joint_values()

    def get_current_pose(self):
        return self.move_group.get_current_pose().pose

    def check_current_pose(self, pose_goal):
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def get_manipulability(self, joint_values):
        move_group = self.move_group

        jacobian_matrix = move_group.get_jacobian_matrix(joint_values)
        # jacobian_matrix.shape is '6 x joints'

        A_mat = np.matmul(jacobian_matrix, np.transpose(jacobian_matrix))
        # A_mat.shape is '6 x 6'
        eigen_values, eigen_vectors = np.linalg.eig(A_mat)

        # [ Manipulability ]
        # {Method 1} >= 1 (Smaller is better)
        # The ratio of longest and shortest axes
        # condition_number = np.max(eigen_values) / np.min(eigen_values)
        # ratio = np.sqrt(condition_number)

        # {Method 2} >= 1 (Smaller is better)
        # Use condition number of A as manipulability value
        # condition_number = np.max(eigen_values) / np.min(eigen_values)

        # {Method 3}
        # proportional to volume of ellipsoid
        # mu = np.sqrt(np.prod(eigen_values))

        # {Method 4}: MoveIt CPP Version <= 1, >=0 (Bigger is better)
        # **Ref**: http://docs.ros.org/en/jade/api/moveit_core/html/classkinematics__metrics_1_1KinematicsMetrics.html#a8bc0ff4bbb402031460232dfe9a1f18d
        # Get the manipulability = sigma_min/sigma_max where sigma_min and sigma_max are
        # the smallest and largest singular values of the Jacobian matrix J.
        # (The singular values in S are square roots of eigenvalues from J*JT or JT*J.)
        mu = np.sqrt(np.min(eigen_values) / np.max(eigen_values))

        return mu

    def plan_cartesian_path(self, pose):
        waypoints = [pose]
        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        return plan, fraction

    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        # BEGIN_SUB_TUTORIAL display_trajectory
        #
        # Displaying a Trajectory
        # ^^^^^^^^^^^^^^^^^^^^^^^
        # You can ask RViz to visualize a plan (aka trajectory) for you. But the
        # group.plan() method does this automatically so this is not that useful
        # here (it just displays the same trajectory again):
        #
        # A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        # We populate the trajectory_start with our current robot state to copy over
        # any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

        # END_SUB_TUTORIAL

    def execute_plan(self, plan):
        self.move_group.execute(plan, wait=True)
        self.move_group.stop()

    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):
        box_name = self.box_name
        scene = self.scene

        # BEGIN_SUB_TUTORIAL wait_for_scene_update
        #
        # Ensuring Collision Updates Are Receieved
        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        # If the Python node dies before publishing a collision object update message, the message
        # could get lost and the box will not appear. To ensure that the updates are
        # made, we wait until we see the changes reflected in the
        # ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        # For the purpose of this self.moveit, we call this function after adding,
        # removing, attaching or detaching an object in the planning scene. We then wait
        # until the updates have been made or ``timeout`` seconds have passed
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False

    def add_box(self, timeout=4):
        box_name = self.box_name
        scene = self.scene

        # First, we will create a box in the planning scene at the location of the left finger:
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "panda_leftfinger"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.07  # slightly above the end effector
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):
        box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        # For the Panda robot, we set ``grasping_group = 'hand'``.
        # If you are using a different robot, you should change this value
        # to the name of your end effector group name.
        grasping_group = "hand"
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=True, box_is_known=False, timeout=timeout
        )

    def detach_box(self, timeout=4):
        box_name = self.box_name
        scene = self.scene
        eef_link = self.eef_link

        # We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=box_name)

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_known=True, box_is_attached=False, timeout=timeout
        )

    def remove_box(self, timeout=4):
        box_name = self.box_name
        scene = self.scene

        # **Note:** The object must be detached before we can remove it from the world
        scene.remove_world_object(box_name)

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=False, box_is_known=False, timeout=timeout
        )


class WorkspaceSpreader:
    def __init__(self, initial_pose):
        position = self.position_tuple_from_pose(initial_pose)
        self.open_list = set()
        self.closed = set()
        self.open_list.add(position)

        self.dx = 0.1
        self.dy = 0.1
        self.dz = 0.1

        # Range
        self.x_min = self.x_max = position[0]
        self.y_min = self.y_max = position[1]
        self.z_min = self.z_max = position[2]

    def is_available(self):
        return len(self.open_list) > 0

    def next(self):
        position = self.open_list.pop()
        self.closed.add(position)
        return position

    def mark_in_workspace(self, pose):
        position = self.position_tuple_from_pose(pose)
        self.closed.add(position)
        for p in self.around(position):
            if p not in self.closed:
                # If you try to add an already existing element to a Set, it won't do anything.
                self.open_list.add(p)
                self.update_range(p)

    def update_range(self, position):
        x, y, z = position
        if self.x_min > x:
            self.x_min = x
        if self.x_max < x:
            self.x_max = x
        if self.y_min > y:
            self.y_min = y
        if self.y_max < y:
            self.y_max = y
        if self.z_min > z:
            self.z_min = z
        if self.z_max < z:
            self.z_max = z

    def around(self, position):
        for dx in [self.dx, 0.0, -self.dx]:
            for dy in [self.dy, 0.0, -self.dy]:
                # for dz in [self.dz, 0.0, -self.dz]:
                #     new_p = (position[0] + dx, position[1] + dy, position[2] + dz)
                new_p = (position[0] + dx, position[1] + dy, 0.7)
                if new_p != position:
                    yield new_p

    @staticmethod
    def position_tuple_from_pose(pose):
        return (pose.position.x, pose.position.y, pose.position.z)


class Recorder:
    def __init__(self):
        rospy.init_node("manipulability_recorder", anonymous=True)
        print ""
        print "----------------------------------------------------------"
        print " Manipulability Recorder"
        print "----------------------------------------------------------"
        print ""
        self._frame_id = "panda_link0"
        self._group_name = "panda_arm"
        self._ee_name = "panda_hand"
        # self._frame_id = "base_footprint"
        # self._group_name = "right_arm"

        self.moveit = MoveGroupPythonInteface(self._group_name, self._ee_name)
        self.ws = WorkspaceSpreader(self.moveit.get_current_pose())

        # Marker
        self.pub = rospy.Publisher(
            "rviz_visual_tools", visualization_msgs.msg.MarkerArray, queue_size=1
        )

        # Markers
        p = geometry_msgs.msg.Point

        def make_maker():
            m = visualization_msgs.msg.Marker()
            m.header.frame_id = self._frame_id
            m.header.stamp = rospy.Time()
            m.action = visualization_msgs.msg.Marker.ADD
            return m

        # Target
        self.tg_marker_x = make_maker()
        self.tg_marker_x.type = visualization_msgs.msg.Marker.ARROW
        self.tg_marker_x.lifetime.secs = 30
        self.tg_marker_x.id = 200
        self.tg_marker_x.scale.x = 0.15  # arrow length
        self.tg_marker_x.scale.y = 0.015  # arrow width
        self.tg_marker_x.scale.z = 0.03  # arrow height
        self.tg_marker_x.color.a = 0.6
        self.tg_marker_x.color.r = 0.93
        self.tg_marker_x.color.g = 0.18
        self.tg_marker_x.color.b = 0.93
        self.tg_marker_z = make_maker()
        self.tg_marker_z.type = visualization_msgs.msg.Marker.ARROW
        self.tg_marker_z.lifetime.secs = 30
        self.tg_marker_z.id = 201
        self.tg_marker_z.scale.x = 0.15  # arrow length
        self.tg_marker_z.scale.y = 0.015  # arrow width
        self.tg_marker_z.scale.z = 0.03  # arrow height
        self.tg_marker_z.color.a = 0.6
        self.tg_marker_z.color.r = 0.0
        self.tg_marker_z.color.g = 0.0
        self.tg_marker_z.color.b = 1.0

        # Manipulability
        self.m_marker = make_maker()
        self.m_marker.type = visualization_msgs.msg.Marker.ARROW
        self.m_marker.scale.x = 0.1  # arrow length
        self.m_marker.scale.y = 0.015  # arrow width
        self.m_marker.scale.z = 0.03  # arrow height
        # ID_manip: 300 ~

        # Out of workspace
        self.o_marker = make_maker()
        self.o_marker.type = visualization_msgs.msg.Marker.ARROW
        self.o_marker.scale.x = 0.1  # arrow length
        self.o_marker.scale.y = 0.015  # arrow width
        self.o_marker.scale.z = 0.03  # arrow height
        self.o_marker.color.a = 0.7
        self.o_marker.color.r = 0.4
        self.o_marker.color.g = 0.4
        self.o_marker.color.b = 0.4
        # ID_manip: 400 ~

    def manip_color(self, manip):
        m = min(1.0, manip * 5.0)
        return std_msgs.msg.ColorRGBA(r=1.0 - m, g=m, b=0.0, a=0.5)

    @staticmethod
    def quat_msg_rotation(quat, rad, axis):
        quat_array = quaternion_multiply(
            np.array([quat.x, quat.y, quat.z, quat.w]),
            quaternion_about_axis(rad, axis),
        )
        new_quat = geometry_msgs.msg.Quaternion()
        new_quat.x = quat_array[0]
        new_quat.y = quat_array[1]
        new_quat.z = quat_array[2]
        new_quat.w = quat_array[3]
        return new_quat

    @staticmethod
    def get_last_joint_from_plan(plan):
        traj = plan.joint_trajectory.points
        joints = traj[-1].positions if len(traj) > 0 else []
        return joints

    def try_target(self, pose, count):
        position = pose.position
        quat = pose.orientation
        try:
            self.tg_marker_x.pose = copy.deepcopy(pose)
            self.tg_marker_z.pose = copy.deepcopy(pose)
            self.tg_marker_z.pose.orientation = self.quat_msg_rotation(
                quat, np.radians(-90), (0, 1, 0)
            )
            markers = visualization_msgs.msg.MarkerArray()
            markers.markers.append(self.tg_marker_x)
            markers.markers.append(self.tg_marker_z)
            self.pub.publish(markers)

            plan, fraction = self.moveit.plan_cartesian_path(copy.deepcopy(pose))
            joints = self.get_last_joint_from_plan(plan)

            self.moveit.execute_plan(plan)
            success = self.moveit.check_current_pose(pose)
            # self.moveit.force_set_joint_values(joints)
            # success = self.moveit.go_to_pose_goal(position, quat)  # <- slow

            marker_quat = self.quat_msg_rotation(quat, np.radians(-90), (0, 1, 0))
            if success:
                joint_values = list(joints)
                # joint_values = self.moveit.get_current_joints()
                manipulability = self.moveit.get_manipulability(joint_values)
                print "[count: %4d] %.5f" % (count - 300, manipulability)

                self.m_marker.id = count
                self.m_marker.pose.position = position
                self.m_marker.pose.orientation = marker_quat
                self.m_marker.color = self.manip_color(manipulability)
                markers.markers.append(self.m_marker)
            else:
                print "out of workspace"
                print "[count: %4d] out of workspace" % (count - 300)
                self.o_marker.id = count
                self.o_marker.pose.position = position
                self.o_marker.pose.orientation = marker_quat
                markers.markers.append(self.o_marker)

            self.pub.publish(markers)
            return success
        except rospy.ROSInterruptException:
            return
        except KeyboardInterrupt:
            return

    def main(self):
        count = 300
        while self.ws.is_available():
            # print "============ Press `Enter` to start recording ..."
            # raw_input()
            position = self.ws.next()

            p = geometry_msgs.msg.Pose()
            p.position.x = position[0]
            p.position.y = position[1]
            p.position.z = position[2]
            p.orientation.w = 1.0
            p.orientation = self.quat_msg_rotation(
                p.orientation, np.radians(90), (0, 1, 0)
            )

            if self.try_target(p, count):
                self.ws.mark_in_workspace(p)
            count += 1


if __name__ == "__main__":
    r = Recorder()
    r.main()

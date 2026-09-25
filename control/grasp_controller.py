import pybullet as p
import math
from control.trajectory import TrajectoryController


class GraspController:
    def __init__(self, robot, simulation):
        self.robot = robot
        self.simulation = simulation
        self.trajectory = TrajectoryController(robot, simulation)
        # constraint_id is the attachment, attached_object_id is the cube itself
        self.constraint_id = None
        self.attached_object_id = None

    def pick(self, cube):
        """Pick up a cube and return the ID of its attachment constraint."""

        object_id = cube.id
        if object_id is None:
            raise ValueError(
                "The cube must be spawned before it can be picked up")
        if self.constraint_id is not None:
            raise RuntimeError(
                "Release the currently attached object before picking another")

        print("Starting pick sequence")

        print("Opening gripper")
        self.robot.open_gripper()
        self.simulation.run_for(0.5)
        cube_position, _ = p.getBasePositionAndOrientation(object_id)
        above_position = [
            cube_position[0],
            cube_position[1],
            cube_position[2] + 0.25
        ]

        print("Moving above cube")

        self.trajectory.move_to(
            above_position
        )
        # this time the grasp frame goes 5 mm above the cube's center, not the ground
        grasp_position = [
            cube_position[0],
            cube_position[1],
            cube_position[2] + 0.005
        ]

        print("Moving to grasp position")

        self.trajectory.move_to(
            grasp_position, minimum_duration=4.0
        )
        if math.dist(self.robot.get_end_effector_pose()[0], grasp_position) > 0.015:
            raise RuntimeError(
                "Pickup failed: the gripper did not reach the cube")

        print("Closing gripper")
        self.robot.close_gripper()

        self.simulation.run_for(1)

        print("Attaching cube")
        self.attach_object(object_id)

        self.simulation.run_for(0.25)

        print("Lifting object")
        self.trajectory.move_to(
            above_position, minimum_duration=4.0
        )

        print("Pick sequence finished")

        return self.constraint_id

    def attach_object(self, object_id):

        if self.constraint_id is not None:
            raise RuntimeError("An object is already attached")
        gripper_position, gripper_orientation = (
            self.robot.get_end_effector_pose()
        )
        object_position, object_orientation = (
            p.getBasePositionAndOrientation(object_id)
        )
        # converts world coordinates to gripper coordinates
        inverse_gripper_pose = p.invertTransform(
            gripper_position,
            gripper_orientation
        )
        relative_position, relative_orientation = p.multiplyTransforms(
            inverse_gripper_pose[0],
            inverse_gripper_pose[1],
            object_position,
            object_orientation
        )

        contacts = p.getContactPoints(self.robot.robot_id, object_id)
        # c[3] is the robot link, c[8] is contact distance, c[9] is contact force
        touching_fingers = {c[3]
                            for c in contacts if c[8] <= 0.001 and c[9] > 0}
        # issubset checks that both fingers (9 and 10) are in the set
        # the cube's center also has to be within 4 cm of the grasp frame
        if not {9, 10}.issubset(touching_fingers) or math.dist(
            relative_position, [0, 0, 0]
        ) > 0.04:
            raise RuntimeError(
                "Pickup failed: cube must be centered and touching both fingers")

        # makes a fixed connection and saves its ID so we can remove it later
        self.constraint_id = p.createConstraint(
            parentBodyUniqueId=self.robot.robot_id,
            parentLinkIndex=self.robot.END_EFFECTOR_LINK_INDEX,
            childBodyUniqueId=object_id,
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=relative_position,
            childFramePosition=[0, 0, 0],
            parentFrameOrientation=relative_orientation,
            childFrameOrientation=[0, 0, 0, 1]
        )

        p.changeConstraint(self.constraint_id, maxForce=500)
        self.attached_object_id = object_id

        return self.constraint_id

    def place(self, target_position):
        """Place the attached object at a target position."""

        if self.constraint_id is None:
            raise RuntimeError(
                "Pick up an object before attempting to place it")

        above_position = [
            target_position[0],
            target_position[1],
            target_position[2] + 0.28
        ]
        # use its actual offset so the cube, not just the gripper, lands on target
        object_position, _ = p.getBasePositionAndOrientation(
            self.attached_object_id)
        gripper_position, _ = self.robot.get_end_effector_pose()
        lower, upper = p.getAABB(self.attached_object_id)
        half_height = (upper[2] - lower[2]) / 2
        offset = [a - b for a, b in zip(object_position, gripper_position)]
        # assumes the held orientation stays the same and the floor is at z = 0
        # leaves the bottom of the cube about 5 mm above the floor before release
        release_position = [
            target_position[0] - offset[0],
            target_position[1] - offset[1],
            half_height + 0.005 - offset[2]
        ]

        print("Moving above target")
        self.trajectory.move_to(above_position)

        print("Lowering object into target")
        self.trajectory.move_to(release_position, minimum_duration=4.0)

        print("Opening gripper and releasing object")
        self.release_object()
        self.robot.open_gripper()
        self.simulation.run_for(0.5)
        self.simulation.run_for(1)

        print("Retreating from target")
        self.trajectory.move_to(above_position)

    def release_object(self):
        """Remove the current attachment and let physics control the object."""

        if self.constraint_id is None:
            return False

        p.removeConstraint(self.constraint_id)
        self.constraint_id = None
        self.attached_object_id = None

        return True

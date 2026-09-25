import pybullet as p


class PandaRobot:

    END_EFFECTOR_LINK_INDEX = 11  # the grasp frame between the fingers
    MAX_JOINT_SPEED = 0.5  # radians per second

    def __init__(self):
        self.robot_id = None

    def load(self):

        self.robot_id = p.loadURDF(
            "franka_panda/panda.urdf",
            [0, 0, 0],
            useFixedBase=True
        )

        self.reset_pose()

        print("Panda loaded")

    def reset_pose(self):

        home_pose = [
            0,
            -0.5,
            0,
            -2.0,
            0,
            1.5,
            0.8
        ]

        for i, angle in enumerate(home_pose):

            # sets the starting pose instantly, only used when loading the robot
            p.resetJointState(
                self.robot_id,
                i,
                angle
            )

        self.command_joint_positions(home_pose)

    def move_to_position(self, position):
        self.command_joint_positions(self.solve_ik(position))

    def solve_ik(self, position):
        """Calculate an arm pose without commanding a sudden movement."""

        orientation = p.getQuaternionFromEuler(
            [
                0,
                3.14159,
                0
            ]
        )

        joints = p.calculateInverseKinematics(
            self.robot_id,
            endEffectorLinkIndex=self.END_EFFECTOR_LINK_INDEX,
            targetPosition=position,
            targetOrientation=orientation,
            maxNumIterations=1000,
            residualThreshold=1e-5
        )

        return joints[:7]  # only the arm joints, not the fingers

    def get_joint_positions(self):
        return [state[0] for state in p.getJointStates(self.robot_id, range(7))]

    def command_joint_positions(self, positions):
        for i, angle in enumerate(positions):

            p.setJointMotorControl2(
                self.robot_id,
                i,
                p.POSITION_CONTROL,
                targetPosition=angle,
                force=500,
                maxVelocity=self.MAX_JOINT_SPEED
            )

    def open_gripper(self):

        for joint in [9, 10]:

            p.setJointMotorControl2(
                self.robot_id,
                joint,
                p.POSITION_CONTROL,
                0.04,
                force=10,
                maxVelocity=0.04
            )

    def get_end_effector_pose(self):
        """Return the world position and orientation of the grasp frame."""

        link_state = p.getLinkState(
            self.robot_id,
            self.END_EFFECTOR_LINK_INDEX,
            computeForwardKinematics=True
        )

        return link_state[4], link_state[5]

    def close_gripper(self):

        for joint in [9, 10]:

            p.setJointMotorControl2(
                self.robot_id,
                joint,
                p.POSITION_CONTROL,
                0,
                force=10,
                maxVelocity=0.04
            )

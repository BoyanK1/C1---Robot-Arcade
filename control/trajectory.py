import math


class TrajectoryController:
    """Time-scaled joint motion with gentle starts and stops."""

    MAX_SPEED = 0.4  # rad/s, below the motor's safety cap
    MAX_ACCELERATION = 0.5  # rad/s squared

    def __init__(self, robot, simulation):
        self.robot = robot
        self.simulation = simulation

    def move_to(self, position, minimum_duration=3.0):
        start = self.robot.get_joint_positions()
        goal = self.robot.solve_ik(position)
        delta = [b - a for a, b in zip(start, goal)]
        travel = max(abs(value) for value in delta)
        distance = math.dist(self.robot.get_end_effector_pose()[0], position)
        # 1.875 and 10/sqrt(3) come from the curve's peak speed and acceleration
        duration = max(
            minimum_duration,
            1.875 * travel / self.MAX_SPEED,
            math.sqrt((10 / math.sqrt(3)) * travel / self.MAX_ACCELERATION),
            1.875 * distance / 0.12,
        )
        steps = max(1, math.ceil(duration / self.simulation.FRAME_TIME))
        for step in range(1, steps + 1):
            t = step / steps
            # this curve starts gently, speeds up in the middle, then slows down
            blend = 10 * t**3 - 15 * t**4 + 6 * t**5
            self.robot.command_joint_positions([
                angle + blend * offset for angle, offset in zip(start, delta)
            ])
            self.simulation.step()

        self.simulation.run_for(0.4)

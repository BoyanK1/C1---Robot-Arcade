class IKController:

    def __init__(self, robot):
        self.robot = robot

    def move_to(self, position):

        # the game uses TrajectoryController instead so the movement stays gradual
        self.robot.move_to_position(position)

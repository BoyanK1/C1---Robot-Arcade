from simulation.simulation import Simulation
from robot.panda_robot import PandaRobot
from game.game_manager import GameManager


simulation = Simulation()

simulation.start()


robot = PandaRobot()
robot.load()


game = GameManager(robot, simulation)
game.setup()
game.play()


# keeps physics and the window running after the round ends
while True:
    simulation.step()

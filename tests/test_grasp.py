import math
import unittest

import pybullet as p

from game.game_manager import GameManager
from robot.panda_robot import PandaRobot
from simulation.simulation import Simulation


class GraspTests(unittest.TestCase):
    def setUp(self):
        self.simulation = Simulation(gui=False)
        self.simulation.start()
        self.robot = PandaRobot()
        self.robot.load()
        self.game = GameManager(self.robot, self.simulation)
        self.game.setup()

    def tearDown(self):
        p.disconnect(self.simulation.client)

    def test_attachment_across_open_air_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, "touching both fingers"):
            self.game.controller.attach_object(self.game.cubes[0].id)
        self.assertEqual(p.getNumConstraints(), 0)
        self.assertIsNone(self.game.controller.attached_object_id)

    def test_every_pick_has_two_finger_contacts_before_attachment(self):
        controller = self.game.controller
        attach = controller.attach_object
        verified = []

        def check_then_attach(body):
            contacts = p.getContactPoints(self.robot.robot_id, body)
            fingers = {c[3] for c in contacts if c[8] <= 0.001 and c[9] > 0}
            self.assertTrue({9, 10}.issubset(fingers))
            cube_position, _ = p.getBasePositionAndOrientation(body)
            grasp_position, _ = self.robot.get_end_effector_pose()
            self.assertLess(math.dist(cube_position, grasp_position), 0.04)
            result = attach(body)
            verified.append(body)
            return result

        controller.attach_object = check_then_attach
        self.assertEqual(self.game.play(), 3)
        self.assertEqual(len(verified), 3)
        self.assertEqual(p.getNumConstraints(), 0)


if __name__ == "__main__":
    unittest.main()

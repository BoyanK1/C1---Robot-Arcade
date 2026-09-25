import pybullet as p

from control.grasp_controller import GraspController
from objects.cube import Cube
from objects.target_zone import TargetZone


class GameManager:

    COLORS = {
        "red": [1, 0.1, 0.1, 1],
        "green": [0.1, 0.8, 0.2, 1],
        "blue": [0.1, 0.3, 1, 1]
    }

    CUBE_POSITIONS = [
        [0.40, -0.20, 0.05],
        [0.45, 0.00, 0.05],
        [0.40, 0.20, 0.05]
    ]

    TARGET_POSITIONS = [
        [0.65, -0.20, 0.006],
        [0.65, 0.00, 0.006],
        [0.65, 0.20, 0.006]
    ]

    def __init__(self, robot, simulation):
        self.simulation = simulation
        self.controller = GraspController(robot, simulation)
        self.cubes = []
        self.targets = []
        self.score = 0
        self.scored_cube_ids = set()

    def setup(self):
        """Spawn one cube and matching target for every configured color."""

        for (color, rgba), cube_position, target_position in zip(
            self.COLORS.items(),
            self.CUBE_POSITIONS,
            self.TARGET_POSITIONS
        ):
            cube = Cube(color, rgba)
            cube.spawn(cube_position)

            target = TargetZone(color, rgba)
            target.spawn(target_position)

            self.cubes.append(cube)
            self.targets.append(target)

        self.simulation.run_for(0.5)

    def play(self):
        """Pick and place every cube into its matching target."""

        print("Starting Robot Arcade")

        for cube, target in zip(self.cubes, self.targets):
            print(f"\nPlacing {cube.color} cube")
            self.controller.pick(cube)
            self.controller.place(target.position)
            self._score_placement(cube, target)

        print(f"\nRound complete. Final score: {self.score}/{len(self.cubes)}")

        return self.score

    def _score_placement(self, cube, target):
        # stops the same cube getting counted twice
        if cube.id in self.scored_cube_ids:
            return False

        cube_position, _ = p.getBasePositionAndOrientation(cube.id)
        color_matches = cube.color == target.color
        is_inside = target.contains(cube_position)

        if color_matches and is_inside:
            self.score += 1
            self.scored_cube_ids.add(cube.id)
            print(f"Point scored! Score: {self.score}")
            return True

        print(f"No point: {cube.color} cube missed the {target.color} target")
        return False

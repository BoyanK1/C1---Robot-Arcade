import pybullet as p


class Cube:

    HALF_EXTENT = 0.03  # half the side length, so the whole cube is 6 cm wide

    def __init__(self, color="red", rgba_color=None):

        self.id = None
        self.color = color
        self.rgba_color = rgba_color or [1, 0, 0, 1]

    def spawn(self, position):

        visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[self.HALF_EXTENT] * 3,
            rgbaColor=self.rgba_color
        )

        collision = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[self.HALF_EXTENT] * 3
        )

        self.id = p.createMultiBody(
            0.1,  # mass in kg
            collision,
            visual,
            position
        )

        print(f"{self.color.capitalize()} cube spawned")

        return self.id

import pybullet as p


class TargetZone:

    HALF_EXTENT = 0.06
    HEIGHT = 0.005  # half its thickness, used by halfExtents

    def __init__(self, color, rgba_color):
        self.id = None
        self.color = color
        self.rgba_color = [*rgba_color[:3], 0.45]
        self.position = None

    def spawn(self, position):
        """Create a thin, non-colliding square that marks a drop zone."""

        visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[self.HALF_EXTENT, self.HALF_EXTENT, self.HEIGHT],
            rgbaColor=self.rgba_color
        )

        self.id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=-1,  # visual marker only, nothing to bump into
            baseVisualShapeIndex=visual,
            basePosition=position
        )
        self.position = position

        print(f"{self.color.capitalize()} target spawned")

        return self.id

    def contains(self, object_position):
        """Return whether an object's center is horizontally inside the zone."""

        if self.position is None:
            return False

        # checks the cube center in x and y, not its height or all its corners
        return (
            abs(object_position[0] - self.position[0]) <= self.HALF_EXTENT
            and abs(object_position[1] - self.position[1]) <= self.HALF_EXTENT
        )

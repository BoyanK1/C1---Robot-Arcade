import pybullet as p
import pybullet_data
import time
import math


class Simulation:

    FRAME_TIME = 1 / 60

    def __init__(self, gui=True):

        self.gui = gui
        self.client = p.connect(p.GUI if gui else p.DIRECT)
        # we step physics ourselves, with 4 smaller updates inside each frame
        p.setRealTimeSimulation(0)
        p.setTimeStep(self.FRAME_TIME)
        p.setPhysicsEngineParameter(numSubSteps=4)
        if gui:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)
            # draws once per frame so extra PyBullet calls don't slow the Mac GUI down
            p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING, 1)

        p.setAdditionalSearchPath(
            pybullet_data.getDataPath()
        )

        p.setGravity(
            0,
            0,
            -9.81
        )

    def start(self):

        p.loadURDF(
            "plane.urdf"
        )

        p.resetDebugVisualizerCamera(
            cameraDistance=2,
            cameraYaw=60,
            cameraPitch=-35,
            cameraTargetPosition=[0, 0, 0.3]
        )

        print("Simulation started")

    def step(self):

        started = time.perf_counter()
        if self.gui:
            p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING, 1)
        p.stepSimulation(physicsClientId=self.client)
        if self.gui:
            # sleeps only for the time left in the frame, not another whole frame
            time.sleep(max(0, self.FRAME_TIME - (time.perf_counter() - started)))

    def run_for(self, duration):
        """Advance the physics simulation for approximately ``duration`` seconds."""

        steps = max(0, math.ceil(duration / self.FRAME_TIME))

        for _ in range(steps):
            self.step()

"""Sample Webots controller for the humanoid sprint benchmark."""

from controller import Robot, Motion
from controller import Supervisor
from controller import Node
import random
import decimal
import math


class Sprinter(Supervisor):
    """Make the NAO robot run as fast as possible."""

    def initialize(self):
        """Get device pointers, enable sensors and set robot initial pose."""
        # This is the time step (ms) used in the motion file.
        self.timeStep = 40
        # Get pointers to the shoulder motors.
        self.RShoulderPitch = self.getDevice('RShoulderPitch')
        self.LShoulderPitch = self.getDevice('LShoulderPitch')
        # Move the arms down.
        self.RShoulderPitch.setPosition(1.1)
        self.LShoulderPitch.setPosition(1.1)

        # # Get pointers to the 12 motors of the legs (not used).
        # self.RHipYawPitch = self.getDevice('RHipYawPitch')  # not used in forward.motion
        # self.LHipYawPitch = self.getDevice('LHipYawPitch')  # not used in forward.motion
        # self.RHipRoll = self.getDevice('RHipRoll')
        # self.LHipRoll = self.getDevice('LHipRoll')
        # self.RHipPitch = self.getDevice('RHipPitch')
        # self.LHipPitch = self.getDevice('LHipPitch')
        # self.RKneePitc = self.getDevice('RKneePitch')
        # self.LKneePitch = self.getDevice('LKneePitch')
        # self.RAnklePitch = self.getDevice('RAnklePitch')
        # self.LAnklePitch = self.getDevice('LAnklePitch')
        # self.RAnkleRoll = self.getDevice('RAnkleRoll')
        # self.LAnkleRoll = self.getDevice('LAnkleRoll')
        # getting pointer to the 2 shoulder motors

        # # Get pointers to the onboard cameras (not used).
        # self.CameraTop = self.getDevice('CameraTop')
        # self.CameraBottom = self.getDevice('CameraBottom')
        # # Enable the cameras.
        # self.CameraTop.enable(self.timeStep)
        # self.CameraBottom.enable(self.timeStep)

    def run(self):
        initCoordinates = self.getCoordinates()
        numOfCycles = 0
        """Play the forward motion and loop on the walking cycle."""
        walk = Motion('forward.motion')
        walk.setLoop(True)
        walk.play()
        while numOfCycles<5:
            # # This motor is not controlled by forward.motion.
            # self.RHipYawPitch.setPosition(-1)
            # print walk.getTime()  # display the current time of the forward.motion
            if walk.getTime() == 340:  # we reached the end of forward.motion
                walk.setTime(90)  # loop back to the beginning of the walking sequence
                numOfCycles = numOfCycles + 1
                print(self.calculateDistance(initCoordinates, self.getCoordinates()))
            # Perform a simulation step, quit if the simulation is over.
            if self.step(self.timeStep) == -1:
                break
        self.rerunSimulation()

    def rerunSimulation(self):
        Supervisor.simulationReset(self)
        self.run()

    def getCoordinates(self):
        return Node.getPosition(Supervisor.getSelf(self))

    def calculateDistance(self, point1, point2):
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2 + (point1[2] - point2[2])**2)


controller = Sprinter()
controller.initialize()
controller.run()

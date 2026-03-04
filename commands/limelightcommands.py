import commands2
import wpilib
import rev
import time
import math
from constants import AuxConstants
from rev import SparkMax, SparkMaxConfig
from commands2.button import CommandXboxController
import subsystems.limelight 
# 1. Subsystem
class LimelightCommands(commands2.Subsystem):
    def __init__(self, Limelight):
        super().__init__()
        self.limelight = Limelight

    def get_distance(self):
        # distance = (targetHeight - cameraHeight) / tan(cameraAngle + targetAngle)
        targetHeight = 44.0  # inches, change as needed
        cameraHeight = 20.0  # inches, change as needed
        cameraAngle = math.radians(36.25)  # degrees, change as needed
        targetAngle = math.radians(self.limelight.getY())  # getY() returns the vertical angle to the target

        distance = (targetHeight - cameraHeight) / math.tan(cameraAngle + targetAngle)
        return distance

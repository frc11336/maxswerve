import commands2
import wpilib
import rev
import time
import math
from constants import AuxConstants
from rev import SparkMax, SparkMaxConfig
from commands2.button import CommandXboxController
# 1. Subsystem
class ClimberSubsystem(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        # Initialize NEO motor
        self.Climb = SparkMax(AuxConstants.Climb_ID, type=SparkMax.MotorType.kBrushless)


    def Climb_set_speed(self, speed):
        self.Climb.set(speed)
    
    def Climb_stop(self):
        self.Climb_set_speed(0)
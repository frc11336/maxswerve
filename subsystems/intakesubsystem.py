import commands2
import wpilib
import rev
import time
import math
from constants import AuxConstants
from rev import SparkMax, SparkMaxConfig
from commands2.button import CommandXboxController
# 1. Subsystem
class IntakeSubsystem(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        # Initialize NEO motor
        self.Intake = SparkMax(AuxConstants.Intake_ID, type=SparkMax.MotorType.kBrushless)
        self.Intake.setInverted(False)


    def Intake_set_speed(self, speed):
        self.Intake.set(-speed)

    def Intake_stop(self):
        self.Intake_set_speed(0)

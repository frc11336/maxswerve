import commands2
import wpilib
import rev
import time
import math
from constants import AuxConstants
from rev import SparkMax, SparkMaxConfig
from commands2.button import CommandXboxController
# 1. Subsystem
class ShooterSubsystem(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        # Initialize NEO motor
        self.motor_1 = SparkMax(AuxConstants.Shooter_ID_1, type=SparkMax.MotorType.kBrushless)
        self.motor_2 = SparkMax(AuxConstants.Shooter_ID_2, type=SparkMax.MotorType.kBrushless)

        self.feeder = SparkMax(AuxConstants.Feeder_ID, type=SparkMax.MotorType.kBrushed)

    def Shooter_set_speed(self, speed):
        self.motor_1.set(-speed)
        self.motor_2.set(speed)

    def Shooter_stop(self):
        self.Shooter_set_speed(0)

    def Feeder_set_speed(self, speed):
        self.feeder.set(speed)

    def Feeder_stop(self):
        self.Feeder_set_speed(0)


    



            


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
        self.motor1 = SparkMax(AuxConstants.Shooter_ID_1, type=SparkMax.MotorType.kBrushless)
        self.motor2 = SparkMax(AuxConstants.Shooter_ID_2, type=SparkMax.MotorType.kBrushless)
        self.motor1.setInverted(True)  # Invert one motor for proper direction
        self.Shooter = SparkMax.

    def set_speed(self, speed):
        self.Shooter.set(speed)


    def stop(self):
        self.set_speed(0)


    def fire(self, distance):
        x = (-0.0000037 * (distance ^ 2)) + (0.0025 * distance) + 0.736
        self.set_speed(-x)
        print(x)
    



            


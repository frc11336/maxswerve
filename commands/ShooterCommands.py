import commands2
import wpilib
import rev
import time
import math
from constants import AuxConstants
from rev import SparkMax, SparkMaxConfig
from commands2.button import CommandXboxController
from subsystems.shootersubsystem import ShooterSubsystem

class shootercommands():
    def __init__(self):
        self.shooter = ShooterSubsystem()


    def fire(self, distance):
        #x = (-0.0000037 * (distance ^ 2)) + (0.0025 * distance) + 0.736
        force = (0.001812*(distance)) + 0.754
        self.shooter.set_speed(force)
        print("Distance: " + str(distance) + "in")
        print("Force: " + str(force))   
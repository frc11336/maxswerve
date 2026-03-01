import commands2
import wpilib
import rev
import time
import math
from constants import AuxConstants
from rev import SparkMax, SparkMaxConfig
from commands2.button import CommandXboxController
# 1. Subsystem
class ShooterCommands(commands2.Subsystem):
    

    def get_power(distance):
        power = 
import commands2
import wpilib
import rev
import time
import math
from constants import AuxConstants
from rev import SparkMax, SparkMaxConfig
from commands2.button import CommandXboxController
import subsystems.shootersubsystem 
# 1. Subsystem
class LiftCommands(commands2.Subsystem):
    def __init__(self, lift):
        super().__init__()
        self.Lift = lift


    
    def climb(self):
        self.Lift.climb_set_speed(1)
        time.sleep(2)
        self.Lift.climb_stop()
    

        

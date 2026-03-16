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
        self.climb_start_time = 0
        self.is_climbing = False

    def climb(self):
        """Initiates climb sequence (non-blocking)"""
        if not self.is_climbing:
            self.Lift.Climb_set_speed(1)
            self.climb_start_time = wpilib.Timer.getFPGATimestamp()
            self.is_climbing = True
    
    def periodic(self) -> None:
        """Check if it's time to stop climbing"""
        if self.is_climbing:
            # Stop climbing after 2 seconds
            if wpilib.Timer.getFPGATimestamp() - self.climb_start_time >= 2.0:
                self.Lift.Climb_stop()
                self.is_climbing = False
    

        

import commands2
import wpilib
import rev
import time
import math
from constants import AuxConstants
from rev import SparkMax, SparkMaxConfig, ResetMode, PersistMode
from commands2.button import CommandXboxController
# 1. Subsystem
class ClimberSubsystem(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        try:
            # Initialize NEO motor
            self.Climb = SparkMax(AuxConstants.Climb_ID, type=SparkMax.MotorType.kBrushless)
            
            # Configure climber motor
            climb_config = SparkMaxConfig()
            climb_config.inverted(False)
            self.Climb.configure(climb_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
            
            print("ClimberSubsystem initialized successfully")
        except Exception as e:
            print(f"ERROR initializing ClimberSubsystem: {e}")
            import traceback
            traceback.print_exc()
            self.Climb = None


    def Climb_set_speed(self, speed):
        if self.Climb is not None:
            self.Climb.set(speed)
    
    def Climb_stop(self):
        self.Climb_set_speed(0)
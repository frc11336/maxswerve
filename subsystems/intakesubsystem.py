import commands2
import wpilib
import rev
import time
import math
from constants import AuxConstants
from rev import SparkMax, SparkMaxConfig, ResetMode, PersistMode
from commands2.button import CommandXboxController
# 1. Subsystem
class IntakeSubsystem(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        try:
            # Initialize NEO motor
            self.Intake = SparkMax(AuxConstants.Intake_ID, type=SparkMax.MotorType.kBrushless)
            self.intakeon = False
            wpilib.SmartDashboard.putBoolean("Intake On", self.intakeon)
            
            # Configure intake motor
            intake_config = SparkMaxConfig()
            intake_config.inverted(False)
            self.Intake.configure(intake_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
            
            print("IntakeSubsystem initialized successfully")
        except Exception as e:
            print(f"ERROR initializing IntakeSubsystem: {e}")
            import traceback
            traceback.print_exc()
            self.Intake = None


    def Intake_set_speed(self, speed):
        if self.Intake is not None:
            self.Intake.set(-speed)
            self.intakeon = speed != 0
            wpilib.SmartDashboard.putBoolean("Intake On", self.intakeon)

    def Intake_stop(self):
        self.Intake_set_speed(0)

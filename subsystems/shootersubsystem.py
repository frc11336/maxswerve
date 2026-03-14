import commands2
import wpilib
import rev
import time
import math
from constants import AuxConstants
from rev import SparkMax, SparkMaxConfig, ResetMode, PersistMode
from commands2.button import CommandXboxController
# 1. Subsystem
class ShooterSubsystem(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        # Initialize NEO motor
        self.motor_1 = SparkMax(AuxConstants.Shooter_ID_1, type=SparkMax.MotorType.kBrushless)
        self.motor_2 = SparkMax(AuxConstants.Shooter_ID_2, type=SparkMax.MotorType.kBrushless)

        # Configure motor_1 (not inverted)
        motor_1_config = SparkMaxConfig()
        motor_1_config.inverted(False)
        self.motor_1.configure(motor_1_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)

        # Configure motor_2 (inverted and follows motor_1)
        motor_2_config = SparkMaxConfig()
        motor_2_config.inverted(True)
        motor_2_config.follow(self.motor_1)
        self.motor_2.configure(motor_2_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)

        # Configure feeder motor
        self.feeder = SparkMax(AuxConstants.Feeder_ID, type=SparkMax.MotorType.kBrushed)
        feeder_config = SparkMaxConfig()
        feeder_config.inverted(False)
        self.feeder.configure(feeder_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)

    def Shooter_set_speed(self, speed):
        self.motor_1.set(speed)

    def Shooter_stop(self):
        self.Shooter_set_speed(0)

    def Feeder_set_speed(self, speed):
        self.feeder.set(speed)

    def Feeder_stop(self):
        self.Feeder_set_speed(0)
    
    def Shooter_kill(self):
        self.Shooter_stop()
        self.Feeder_stop()


    



            


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

        self.shooteron = False
        self.feederon = False
        wpilib.SmartDashboard.putBoolean("Shooter On", self.shooteron)
        wpilib.SmartDashboard.putBoolean("Feeder On", self.feederon)

        # Initialize NEO motor
        self.motor_1 = SparkMax(AuxConstants.Shooter_ID_1, SparkMax.MotorType.kBrushless)
        #self.motor_2 = SparkMax(AuxConstants.Shooter_ID_2, SparkMax.MotorType.kBrushless)
        self.feeder = SparkMax(AuxConstants.Feeder_ID, SparkMax.MotorType.kBrushless)


    def Shooter_set_speed(self, speed):
        self.motor_1.set(speed)
        #self.motor_2.set(-speed)
        self.shooteron = speed != 0
        wpilib.SmartDashboard.putBoolean("Shooter On", self.shooteron)

    def Shooter_stop(self):
        self.Shooter_set_speed(0)

    def Feeder_set_speed(self, speed):
        self.feeder.set(-speed)
        self.intakeon = speed != 0
        wpilib.SmartDashboard.putBoolean("Intake On", self.intakeon)

    def Feeder_stop(self):
        self.Feeder_set_speed(0)
    
    def Shooter_kill(self):
        self.Shooter_stop()
        self.Feeder_stop()


    



            


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
        self.motor = SparkMax(AuxConstants.Shooter_ID, type=SparkMax.MotorType.kBrushless)


    def set_speed(self, speed):
        self.motor.set(-speed)

    def stop(self):
        self.motor.set(0)

    def fire(self, distance):
        x = (-0.0000037 * (distance ^ 2)) + (0.0025 * distance) + 0.736
        self.motor.set(-x)
    

            

# 2. Inside RobotContainer
class RobotContainer:
    def __init__(self):
        self.shooter = ShooterSubsystem()
        self.driver_controller = CommandXboxController(0)
        self.configureButtonBindings()
        

    def configureButtonBindings(self):
        # Bind A button to run motor while held (whileTrue) or toggle (onTrue)
        self.driver_controller.a().whileTrue(
            commands2.FunctionalCommand(
                lambda: self.shooter.set_speed(0.5), # Start
                lambda: None,                        # Execute
                lambda interrupted: self.shooter.stop(), # End
                lambda: False,                       # IsFinished
                self.shooter
            )
        )

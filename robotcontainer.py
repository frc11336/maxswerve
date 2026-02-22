#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math

import commands2
import wpimath
import wpilib

from commands2.button import CommandXboxController
from wpimath.controller import PIDController, ProfiledPIDControllerRadians
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from rev import SparkMax
from commands2 import cmd, RunCommand
from constants import AuxConstants
from commands2 import cmd
from wpimath.controller import PIDController, ProfiledPIDControllerRadians
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import (
    TrajectoryConfig,
    TrajectoryGenerator,
    TrapezoidProfileRadians,
)
from wpimath.controller import (
    HolonomicDriveController,
    PIDController,
    ProfiledPIDControllerRadians,
)

from constants import AutoConstants, DriveConstants, OIConstants
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.shootersubsystem import ShooterSubsystem
from commands.ShooterCommands import shootercommands
from subsystems.limelight_camera import LimelightCamera

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # The robot's subsystems

        
        self.camera = LimelightCamera("limelight-pickup")  # name of your camera goes in parentheses

        self.robotDrive = DriveSubsystem()
        self.Shooter = ShooterSubsystem()
        self.ShooterCommands = shootercommands()
        # The driver's controller
        # Using commands2 instead of wpilib
        #self.driverController = wpilib.XboxController(OIConstants.kDriverControllerPort)
        self.distance = 100

        # The driver's controller
        self.driverController = commands2.button.CommandXboxController(
            #OperatorConstants.DRIVER_CONTROLLER_PORT : Directly defining port
            0
        )

        # Configure the button bindings
        self.configureButtonBindings()

        # Configure default commands
        self.robotDrive.setDefaultCommand(
            # The left stick controls translation of the robot.
            # Turning is controlled by the X axis of the right stick.
            commands2.RunCommand(
                lambda: self.robotDrive.drive(
                    -wpimath.applyDeadband(
                        self.driverController.getLeftY(), OIConstants.kDriveDeadband
                    ),
                    -wpimath.applyDeadband(
                        self.driverController.getLeftX(), OIConstants.kDriveDeadband
                    ),
                    -wpimath.applyDeadband(
                        self.driverController.getRightX(), OIConstants.kDriveDeadband
                    ),
                    True,
                    True,
                ),
                self.robotDrive,
            )
        )

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        def turn_to_object():
            x = self.camera.getX()
            print(f"x={x}")
            turn_speed = -0.005 * x
            self.robotDrive.drive(0, 0, turn_speed, False, False)
            # if you want your robot to slowly chase that object... replace this line above with: self.robotDrive.arcadeDrive(0.1, turn_speed)

            self.driverController.start().whileTrue(cmd.run(lambda:turn_to_object()))
           
        def distanceplus():
            self.distance = round(self.distance + 1)
            print("")
            print("")
            print("")
            print("")
            print("")
            print("")
            print("")
            print("Distance:" + str(self.distance) + "in" )
            print("_" * round(self.distance/10))

        def distanceminus():
            self.distance = round(self.distance - 1)
            print("") 
            print("")
            print("")
            print("")
            print("")
            print("")
            print("")
            print("")
            print("Distance:" + str(self.distance) + "in" )
            print("_" * round(self.distance/10))


        self.driverController.y().whileTrue(cmd.runOnce(lambda:distanceplus()))
        self.driverController.x().whileTrue(cmd.runOnce(lambda:distanceminus()))

        self.driverController.rightBumper().onTrue(cmd.runOnce(lambda:self.Shooter.set_speed(80)))
        self.driverController.rightBumper().onFalse(cmd.runOnce(lambda: self.Shooter.stop()))


        self.intake = SparkMax(AuxConstants.Intake_ID, SparkMax.MotorType.kBrushed)
        self.driverController.leftBumper().onTrue(cmd.runOnce(lambda:self.intake.set(1)))
        self.driverController.leftBumper().onFalse(cmd.runOnce(lambda:self.intake.set(0)))

        #self.lift = SparkMax(AuxConstants.Lift_ID, SparkMax.MotorType.kBrushed)
        #self.driverController.x().onTrue(cmd.runOnce(lambda:self.lift.set(1)))
        #self.driverController.x().onTrue(cmd.runOnce(lambda:self.lift.set(0)))
        #self.driverController.y().onTrue(cmd.runOnce(lambda:self.lift.set(-1)))
        #self.driverController.y().onTrue(cmd.runOnce(lambda:self.lift.set(0)))

        self.climb = SparkMax(AuxConstants.Climb_ID, SparkMax.MotorType.kBrushless)
        self.driverController.a().onTrue(cmd.runOnce(lambda:self.climb.set(1)))
        self.driverController.a().onFalse(cmd.runOnce(lambda:self.climb.set(0)))
        self.driverController.b().onTrue(cmd.runOnce(lambda:self.climb.set(-1)))
        self.driverController.b().onFalse(cmd.runOnce(lambda:self.climb.set(0)))

    def disablePIDSubsystems(self) -> None:
        """Disables all ProfiledPIDSubsystem and PIDSubsystem instances.
        This should be called on robot disable to prevent integral windup."""

    def getAutonomousCommand(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        # Create config for trajectory
        config = TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared,
        )
        # Add kinematics to ensure max speed is actually obeyed
        config.setKinematics(DriveConstants.kDriveKinematics)

        # An example trajectory to follow. All units in meters.
        exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            # Start at the origin facing the +X direction
            Pose2d(0, 0, Rotation2d(0)),
            # Pass through these two interior waypoints, making an 's' curve path
            [Translation2d(.1, .1), Translation2d(.2, -.1)],
            # End 3 meters straight ahead of where we started, facing forward
            Pose2d(.3, 0, Rotation2d(0)),
            config,
        )
        def Shoot_5sec() :
            self.Shooter.set_speed(-.5)
            commands2.WaitCommand(5.0)
            self.Shooter.stop()
        
        Shoot_5sec()
        # Constraint for the motion profiled robot angle controller
        kThetaControllerConstraints = TrapezoidProfileRadians.Constraints(
            AutoConstants.kMaxAngularSpeedRadiansPerSecond,
            AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared,
        )

        kPXController = PIDController(1.0, 0.0, 0.0)
        kPYController = PIDController(1.0, 0.0, 0.0)
        kPThetaController = ProfiledPIDControllerRadians(
            1.0, 0.0, 0.0, kThetaControllerConstraints
        )
        kPThetaController.enableContinuousInput(-math.pi, math.pi)

        kPIDController = HolonomicDriveController(
            kPXController, kPYController, kPThetaController
        )

        swerveControllerCommand = commands2.SwerveControllerCommand(
            exampleTrajectory,
            self.robotDrive.getPose,  # Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            # Position controllers
            kPIDController,
            # <--- CHANGED LINE
            lambda desiredStates: self.robotDrive.setModuleStates(desiredStates),
            (self.robotDrive,),
        )

        
        # Reset odometry to the starting pose of the trajectory.
        self.robotDrive.resetOdometry(exampleTrajectory.initialPose())

        # Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(
            cmd.run(
                lambda: self.robotDrive.drive(0, 0, 0, False, False),
                self.robotDrive,
            )
        )


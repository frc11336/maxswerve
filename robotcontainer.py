#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math

import commands2
import wpimath
import wpilib
from wpilib import SmartDashboard

import json
import time

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

from subsystems.limelight import LimelightCamera
from constants import AutoConstants, DriveConstants, OIConstants
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.shootersubsystem import ShooterSubsystem
from subsystems.climbersubsystem import ClimberSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from commands.shootercommands import ShooterCommands
from commands.limelightcommands import LimelightCommands
from commands.liftcommands import LiftCommands
from commands.rotatetoobjectcommand import RotateToObjectCommand
from commands.aimathubcommand import AimAtHubCommand, distance_to_nearest_hub

from pathplannerlib.auto import PathPlannerAuto
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.auto import NamedCommands





class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        try:
            # The robot's subsystems
            print("Initializing DriveSubsystem...")
            self.robotDrive = DriveSubsystem()
            
            print("Initializing ShooterSubsystem...")
            self.Shooter = ShooterSubsystem()
            
            print("Initializing ClimberSubsystem...")
            self.Climb = ClimberSubsystem()
            
            print("Initializing IntakeSubsystem...")
            self.Intake = IntakeSubsystem()
            
            print("Initializing ShooterCommands...")
            self.ShooterCommands = ShooterCommands(self.Shooter)
            
            print("Initializing LiftCommands...")
            self.ClimbCommands = LiftCommands(self.Climb)

            self.aimathub = AimAtHubCommand(self.robotDrive)

            def shoot():
                self.distance = self.cameracommands.get_distance()
                self.ShooterCommands.fire(self.distance)


            print("Registering NamedCommands...")
            NamedCommands.registerCommand("spool", cmd.runOnce(lambda: shoot()))
            NamedCommands.registerCommand("feed", cmd.runOnce(lambda: self.Shooter.Feeder_set_speed(1)))
            NamedCommands.registerCommand("stopshoot", cmd.runOnce(lambda: self.Shooter.Shooter_kill()))
            NamedCommands.registerCommand("climb", cmd.runOnce(lambda: self.Climb.Climb_set_speed(.5)))
            NamedCommands.registerCommand("stopclimb", cmd.runOnce(lambda: self.Climb.Climb_stop()))
            NamedCommands.registerCommand("reverseclimb", cmd.runOnce(lambda: self.Climb.Climb_set_speed(-.5)))
            NamedCommands.registerCommand("intake", cmd.runOnce(lambda: self.Intake.Intake_set_speed(1)))
            NamedCommands.registerCommand("stopintake", cmd.runOnce(lambda: self.Intake.Intake_stop()))
        
            print("Initializing controllers and camera...")
            # The driver's controller
            # Using commands2 instead of wpilib
            #self.driverController = wpilib.XboxController(OIConstants.kDriverControllerPort)
            self.distance = 50  # fallback; overwritten by get_hub_distance() at shoot time
            self.poweroffset = 0
            self.intakespeed = .75

 
            SmartDashboard.putNumber("Power Offset", self.poweroffset)


            # The robot's subsystems
            
            self.camera = LimelightCamera("limelight", self.robotDrive)  # name of your camera goes in parentheses
            self.cameracommands = LimelightCommands(self.camera)

            self.relative = False
            SmartDashboard.putBoolean("field relative", self.relative)

            # The driver's controller
            self.driverController = commands2.button.CommandXboxController (0)
            self.auxilaryController = commands2.button.CommandXboxController(1)

            print("Building AutoChooser...")
            # Build an auto chooser. This will use Commands.none() as the default option.
            self.autoChooser = AutoBuilder.buildAutoChooser()

            print("Putting AutoChooser on SmartDashboard...")
            # Disabled SmartDashboard.putData due to blocking issues
            SmartDashboard.putData("Auto Chooser", self.autoChooser)
            
            print("Configuring button bindings...")
            # Configure the button bindings
            self.configureButtonBindings()

            print("Setting default drive command...")
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
                        self.relative,
                        False,
                    ),
                    self.robotDrive,
                )
            )
            
            print("RobotContainer initialization complete!")
        except Exception as e:
            print(f"CRITICAL ERROR initializing RobotContainer: {e}")
            import traceback
            traceback.print_exc()
            raise

    def getautonomouscommand(self):
        return self.autoChooser.getSelected()

    def updateDistance(self) -> None:
        """Recalculate distance to nearest hub from odometry and publish to SmartDashboard.
        Called periodically from robot.py so the value stays current every loop."""
        try:
            dist_m = distance_to_nearest_hub(self.robotDrive.getPose())
            self.distance = dist_m * 39.3701  # metres → inches
        except Exception:
            self.distance = self.cameracommands.get_distance()
        SmartDashboard.putNumber("Distance to Hub (in)", self.distance)

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """
        def turn_to_object():
            x = self.camera.getX()
            turn_speed = -0.010 * x
            self.robotDrive.rotate(turn_speed)
            # if you want your robot to slowly chase that object... replace this line above with: self.robotDrive.arcadeDrive(0.1, turn_speed)

        def swaprelative():
            self.relative = not self.relative
            SmartDashboard.putBoolean("field relative", self.relative)

        self.driverController.start().onTrue(cmd.runOnce(lambda: swaprelative()))

        def distanceplus():
            self.distance = round(self.distance + 1)

        def distanceminus():
            self.distance = round(self.distance - 1)

        def powerplus():
            self.poweroffset = round(self.poweroffset + 0.01, 2)
            print (self.poweroffset)
            SmartDashboard.putNumber("Power Offset", self.poweroffset)

        def powerminus():
            self.poweroffset = round(self.poweroffset - 0.01, 2)
            print (self.poweroffset)
            SmartDashboard.putNumber("Power Offset", self.poweroffset)

        def getdistance():
            self.updateDistance()
        self.auxilaryController.back().onTrue(cmd.runOnce(lambda:getdistance()))
        
        def shoot():
            self.ShooterCommands.fire(self.distance, self.poweroffset)


        self.auxilaryController.y().onTrue(cmd.runOnce(lambda:powerplus()))
        self.auxilaryController.x().onTrue(cmd.runOnce(lambda:powerminus()))

        #self.auxilaryController.start().onTrue(cmd.runOnce(lambda: self.robotDrive.resetOdometry))
        self.driverController.rightBumper().onTrue(cmd.runOnce(lambda:self.Intake.Intake_set_speed(-self.intakespeed)))
        self.driverController.rightBumper().onFalse(cmd.runOnce(lambda:self.Intake.Intake_stop()))

        self.driverController.leftBumper().onTrue(cmd.runOnce(lambda:self.Intake.Intake_set_speed(self.intakespeed)))
        self.driverController.leftBumper().onFalse(cmd.runOnce(lambda:self.Intake.Intake_stop()))
 
        #self.driverController.a().onTrue(cmd.runOnce(lambda: self.ShooterCommands.fire_Power(self.poweroffset)))
        self.driverController.a().onTrue(cmd.runOnce(lambda: shoot()))
        self.driverController.a().onFalse(cmd.runOnce(lambda: self.Shooter.Shooter_kill()))

        self.driverController.y().onTrue(cmd.runOnce(lambda:self.Climb.Climb_set_speed(.5)))
        self.driverController.y().onFalse(cmd.runOnce(lambda:self.Climb.Climb_stop()))
        self.driverController.b().onTrue(cmd.runOnce(lambda:self.Climb.Climb_set_speed(-.5)))
        self.driverController.b().onFalse(cmd.runOnce(lambda:self.Climb.Climb_stop()))

        #Use X button to aim robot front toward nearest scoring hub
        #self.driverController.x().onTrue(cmd.runOnce(lambda: self.aimathub.aim(self.robotDrive)))

        # Left stick button (L3) — rotate robot to face april tag
        self.driverController.leftStick().onTrue(RotateToObjectCommand(self.robotDrive, self.camera))

        self.driverController.rightStick().onTrue(cmd.runOnce(lambda: self.ShooterCommands.fire_Power(1)))

    

    def disablePIDSubsystems(self) -> None:
        """Disables all ProfiledPIDSubsystem and PIDSubsystem instances.
        This should be called on robot disable to prevent integral windup."""

    """
    def getAutonomousCommand(self) -> commands2.Command:
        ""
        Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        ""
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
            [Translation2d(.3, 0), Translation2d(.6, 0)],
            # End 3 meters straight ahead of where we started, facing forward
            Pose2d(1, 0, Rotation2d(0)),
            config,
        )

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
        """


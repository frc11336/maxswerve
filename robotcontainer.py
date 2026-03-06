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

from pathplannerlib.auto import PathPlannerAuto
from pathplannerlib.auto import AutoBuilder

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # The robot's subsystems
        self.robotDrive = DriveSubsystem()
        self.Shooter = ShooterSubsystem()
        self.Climb = ClimberSubsystem()
        self.Intake = IntakeSubsystem()
        self.ShooterCommands = ShooterCommands(self.Shooter)
        
        # The driver's controller
        # Using commands2 instead of wpilib
        #self.driverController = wpilib.XboxController(OIConstants.kDriverControllerPort)
        self.distance = 50
        self.power = 0.75

        # The robot's subsystems
        
        self.camera = LimelightCamera("limelight")  # name of your camera goes in parentheses
        self.cameracommands = LimelightCommands(self.camera)

        self.relative = False

        # The driver's controller
        self.driverController = commands2.button.CommandXboxController (
            #OperatorConstants.DRIVER_CONTROLLER_PORT : Directly defining port
            0
        )
        # Build an auto chooser. This will use Commands.none() as the default option.
        self.autoChooser = AutoBuilder.buildAutoChooser()

        # Another option that allows you to specify the default auto by its name
        # self.autoChooser = AutoBuilder.buildAutoChooser("My Default Auto")

        SmartDashboard.putData("Auto Chooser", self.autoChooser)
        

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
                    self.relative,
                    False,
                ),
                self.robotDrive,
            )
        )
    
    def getAutonomousCommand(self):
        return self.autoChooser.getSelected()

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
            self.robotDrive.drive(0,0, turn_speed,True, False)
            # if you want your robot to slowly chase that object... replace this line above with: self.robotDrive.arcadeDrive(0.1, turn_speed)

        backButton = self.driverController.button(wpilib.XboxController.Button.kBack)
        backButton.onTrue(cmd.runOnce(lambda: print(self.cameracommands.get_distance())))
     


        def swaprelative():
            self.relative = not self.relative

        self.driverController.start().onTrue(cmd.runOnce(lambda: swaprelative()))

        def distanceplus():
            self.distance = round(self.distance + 1)
            print("Distance: " +str(self.distance) + "in")

        def distanceminus():
            self.distance = round(self.distance - 1)
            print("Distance: " +str(self.distance) + "in")

        def powerplus():
            self.power = round(self.power + 0.05, 2)
            print("Power: " +str(self.power))

        def powerminus():
            self.power = round(self.power - 0.05, 2)
            print("Power: " +str(self.power))

        #self.driverController.y().onTrue(cmd.runOnce(lambda:distanceplus()))
        #self.driverController.x().onTrue(cmd.runOnce(lambda:distanceminus()))
        
        self.driverController.y().onTrue(cmd.runOnce(lambda:powerplus()))
        self.driverController.x().onTrue(cmd.runOnce(lambda:powerminus()))

        self.driverController.rightBumper().onTrue(cmd.runOnce(lambda: self.ShooterCommands.fire_Power(self.power)))
        #self.driverController.rightBumper().onTrue(cmd.runOnce(lambda: self.ShooterCommands.fire(self.distance)))
        self.driverController.rightBumper().onFalse(cmd.runOnce(lambda: self.Shooter.Shooter_stop()))
        self.driverController.rightBumper().onFalse(cmd.runOnce(lambda: self.Shooter.Feeder_stop()))
        #self.shooter = SparkMax(AuxConstants.Shooter_ID, SparkMax.MotorType.kBrushless)
        #self.driverController.rightBumper().onTrue(cmd.runOnce(lambda:self.shooter.set(-0.5)))
        #self.driverController.rightBumper().onFalse(cmd.runOnce(lambda:self.shooter.set()))

 
        self.driverController.leftBumper().onTrue(cmd.runOnce(lambda:self.Intake.Intake_set_speed(1)))
        self.driverController.leftBumper().onFalse(cmd.runOnce(lambda:self.Intake.Intake_stop()))

        #self.lift = SparkMax(AuxConstants.Lift_ID, SparkMax.MotorType.kBrushed)
        #self.driverController.x().onTrue(cmd.runOnce(lambda:self.lift.set(1)))
        #self.driverController.x().onTrue(cmd.runOnce(lambda:self.lift.set(0)))
        #self.driverController.y().onTrue(cmd.runOnce(lambda:self.lift.set(-1)))
        #self.driverController.y().onTrue(cmd.runOnce(lambda:self.lift.set(0)))


        self.driverController.a().onTrue(cmd.runOnce(lambda:self.Climb.Climb_set_speed(.5)))
        self.driverController.a().onFalse(cmd.runOnce(lambda:self.Climb.Climb_stop()))
        self.driverController.b().onTrue(cmd.runOnce(lambda:self.Climb.Climb_set_speed(-.5)))
        self.driverController.b().onFalse(cmd.runOnce(lambda:self.Climb.Climb_stop()))
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
            self.Shooter.Shooter_set_speed(-.5)
            commands2.WaitCommand(5.0)
            self.Shooter.Shooter_stop()
        
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


#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import typing
import navx

from rev import PersistMode, ResetMode, SparkBase, SparkBaseConfig, SparkMax, SparkMaxConfig

import wpilib
from wpilib import SPI, SmartDashboard
from wpilib.shuffleboard import Shuffleboard

import wpimath.controller

from commands2 import Subsystem
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Pose2d, Rotation2d
import wpimath.kinematics
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveModuleState,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
)

from constants import DriveConstants
import swerveutils
from .maxswervemodule import MAXSwerveModule

from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import RobotConfig, PIDConstants
from pathplannerlib.auto import PathPlannerAuto
from wpilib import DriverStation

class DriveSubsystem(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        # Create MAXSwerveModules
        self.frontLeft = MAXSwerveModule(
            DriveConstants.kFrontLeftDrivingCanId,
            DriveConstants.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftChassisAngularOffset,
        )

        self.frontRight = MAXSwerveModule(
            DriveConstants.kFrontRightDrivingCanId,
            DriveConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightChassisAngularOffset,
        )

        self.rearLeft = MAXSwerveModule(
            DriveConstants.kRearLeftDrivingCanId,
            DriveConstants.kRearLeftTurningCanId,
            DriveConstants.kBackLeftChassisAngularOffset,
        )

        self.rearRight = MAXSwerveModule(
            DriveConstants.kRearRightDrivingCanId,
            DriveConstants.kRearRightTurningCanId,
            DriveConstants.kBackRightChassisAngularOffset,
        )

        config = RobotConfig.fromGUISettings()

        # The gyro sensor
        self.gyro = navx.AHRS.create_spi()

        # Print absolute encoder positions for all modules at startup
        print("FL abs encoder:", self.frontLeft.turningEncoder.getPosition())
        print("FR abs encoder:", self.frontRight.turningEncoder.getPosition())
        print("RL abs encoder:", self.rearLeft.turningEncoder.getPosition())
        print("RR abs encoder:", self.rearRight.turningEncoder.getPosition())

        # Slew rate filter variables for controlling lateral acceleration
        self.currentRotation = 0.0
        self.currentTranslationDir = 0.0
        self.currentTranslationMag = 0.0

        self.magLimiter = SlewRateLimiter(DriveConstants.kMagnitudeSlewRate)
        self.rotLimiter = SlewRateLimiter(DriveConstants.kRotationalSlewRate)
        self.prevTime = wpilib.Timer.getFPGATimestamp()

        # Odometry class for tracking robot pose
        self.odometry = SwerveDrive4Odometry(
            DriveConstants.kDriveKinematics,
            -self.gyro.getRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.rearLeft.getPosition(),
                self.rearRight.getPosition(),
            ),
        )

        # Configure the AutoBuilder last
        AutoBuilder.configure(
            self.getPose, # Robot pose supplier
            self.resetOdometry, # Method to reset odometry (will be called if your auto has a starting pose)
            self.getRobotRelativeSpeeds, # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            self.driveRobotRelative, # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also outputs individual module feedforwards
            PPHolonomicDriveController(
                PIDConstants(7.5, 5.0, 0.0),  # Translation PID
                PIDConstants(7.5, 5.0, 0.0)   # Rotation PID
            ), # PPHolonomicDriveController for swerve drives
            config, # The robot configuration
            self.shouldFlipPath, # Supplier to control path flipping based on alliance color
            self # Reference to this subsystem to set requirements
        )

        # Create field visualization
        from wpilib import Field2d
        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)

   

    def periodic(self) -> None:
        # Update the odometry in the periodic block
        self.odometry.update((
            -self.gyro.getRotation2d()),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.rearLeft.getPosition(),
                self.rearRight.getPosition(),
            ))
        
        
        # Update field visualization
        self.field.setRobotPose(self.odometry.getPose())
        
        # Publish pose to SmartDashboard for visualization
        pose = self.odometry.getPose()
        SmartDashboard.putNumber("Robot X", pose.X())
        SmartDashboard.putNumber("Robot Y", pose.Y())
        SmartDashboard.putNumber("Robot Angle", pose.rotation().degrees())
        """
        # Log battery voltage
        voltage = wpilib.RobotController.getBatteryVoltage()
        print(f"Battery Voltage: {voltage:.2f}V")

        # Log CAN status for each Spark Max
        modules = [
            ("FL", self.frontLeft),
            ("FR", self.frontRight),
            ("RL", self.rearLeft),
            ("RR", self.rearRight),
        ]
        
        for name, module in modules:
            try:
                drive_voltage = module.drivingSparkMax.getBusVoltage()
                turn_voltage = module.turningSparkMax.getBusVoltage()
                drive_faults = module.drivingSparkMax.getFaults()
                turn_faults = module.turningSparkMax.getFaults()
                print(f"{name} Drive: {drive_voltage:.2f}V, Faults: {drive_faults}")
                print(f"{name} Turn: {turn_voltage:.2f}V, Faults: {turn_faults}")
            except Exception as e:
                print(f"{name} Error: {e}")
        """
    def getPose(self) -> Pose2d:
        """Returns the currently-estimated pose of the robot.

        :returns: The pose.
        """
        return self.odometry.getPose()

    def resetOdometry(self, pose: Pose2d) -> None:
        """Resets the odometry to the specified pose.

        :param pose: The pose to which to set the odometry.

        """
        self.odometry.resetPosition(
            Rotation2d.fromDegrees(self.gyro.getAngle()),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.rearLeft.getPosition(),
                self.rearRight.getPosition(),
            ),
            pose,
        )

    def drive(
        self,
        xSpeed: float,
        ySpeed: float,
        rot: float,
        fieldRelative: bool,
        rateLimit: bool,
    ) -> None:
        """Method to drive the robot using joystick info.

        :param xSpeed:        Speed of the robot in the x direction (forward).
        :param ySpeed:        Speed of the robot in the y direction (sideways).
        :param rot:           Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative to the
                              field.
        :param rateLimit:     Whether to enable rate limiting for smoother control.
        """

        xSpeedCommanded = xSpeed
        ySpeedCommanded = ySpeed

        if rateLimit:
            # Convert XY to polar for rate limiting
            inputTranslationDir = math.atan2(ySpeed, xSpeed)
            inputTranslationMag = math.hypot(xSpeed, ySpeed)

            # Calculate the direction slew rate based on an estimate of the lateral acceleration
            if self.currentTranslationMag != 0.0:
                directionSlewRate = abs(
                    DriveConstants.kDirectionSlewRate / self.currentTranslationMag
                )
            else:
                directionSlewRate = 500.0
                # some high number that means the slew rate is effectively instantaneous

            currentTime = wpilib.Timer.getFPGATimestamp()
            elapsedTime = currentTime - self.prevTime
            angleDif = swerveutils.angleDifference(
                inputTranslationDir, self.currentTranslationDir
            )
            if angleDif < 0.45 * math.pi:
                self.currentTranslationDir = swerveutils.stepTowardsCircular(
                    self.currentTranslationDir,
                    inputTranslationDir,
                    directionSlewRate * elapsedTime,
                )
                self.currentTranslationMag = self.magLimiter.calculate(
                    inputTranslationMag
                )

            elif angleDif > 0.85 * math.pi:
                # some small number to avoid floating-point errors with equality checking
                # keep currentTranslationDir unchanged
                if self.currentTranslationMag > 1e-4:
                    self.currentTranslationMag = self.magLimiter.calculate(0.0)
                else:
                    self.currentTranslationDir = swerveutils.wrapAngle(
                        self.currentTranslationDir + math.pi
                    )
                    self.currentTranslationMag = self.magLimiter.calculate(
                        inputTranslationMag
                    )

            else:
                self.currentTranslationDir = swerveutils.stepTowardsCircular(
                    self.currentTranslationDir,
                    inputTranslationDir,
                    directionSlewRate * elapsedTime,
                )
                self.currentTranslationMag = self.magLimiter.calculate(0.0)

            self.prevTime = currentTime

            xSpeedCommanded = self.currentTranslationMag * math.cos(
                self.currentTranslationDir
            )
            ySpeedCommanded = self.currentTranslationMag * math.sin(
                self.currentTranslationDir
            )
            self.currentRotation = self.rotLimiter.calculate(rot)

        else:
            self.currentRotation = rot

        # Convert the commanded speeds into the correct units for the drivetrain
        xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond
        ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond
        rotDelivered = self.currentRotation * DriveConstants.kMaxAngularSpeed

        swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedDelivered,
                ySpeedDelivered,
                rotDelivered,
                Rotation2d.fromDegrees(self.gyro.getAngle()),
            )
            if fieldRelative
            else ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered)
        )
        fl, fr, rl, rr = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond
        )
        self.frontLeft.setDesiredState(fl)
        self.frontRight.setDesiredState(fr)
        self.rearLeft.setDesiredState(rl)
        self.rearRight.setDesiredState(rr)

    def setX(self) -> None:
        """Sets the wheels into an X formation to prevent movement."""
        self.frontLeft.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))
        self.frontRight.setDesiredState(
            SwerveModuleState(0, Rotation2d.fromDegrees(-45))
        )
        self.rearLeft.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.rearRight.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))

    def setModuleStates(
        self,
        desiredStates: typing.Sequence[SwerveModuleState],
    ) -> None:
        """Sets the swerve ModuleStates.

        :param desiredStates: The desired SwerveModule states.
        """
        states_tuple = (desiredStates[0], desiredStates[1], desiredStates[2], desiredStates[3])
        fl, fr, rl, rr = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            states_tuple, DriveConstants.kMaxSpeedMetersPerSecond
        )
        self.frontLeft.setDesiredState(fl)
        self.frontRight.setDesiredState(fr)
        self.rearLeft.setDesiredState(rl)
        self.rearRight.setDesiredState(rr)

    def resetEncoders(self) -> None:
        """Resets the drive encoders to currently read a position of 0."""
        self.frontLeft.resetEncoders()
        self.rearLeft.resetEncoders()
        self.frontRight.resetEncoders()
        self.rearRight.resetEncoders()

    def zeroHeading(self) -> None:
        """Zeroes the heading of the robot."""
        self.gyro.reset()

    def getHeading(self) -> float:
        """Returns the heading of the robot.

        :returns: the robot's heading in degrees, from -180 to 180
        """
        return Rotation2d.fromDegrees(self.gyro.getAngle()).degrees()

    def getTurnRate(self) -> float:
        """Returns the turn rate of the robot.

        :returns: The turn rate of the robot, in degrees per second
        """
        return self.gyro.getRate() * (-1.0 if DriveConstants.kGyroReversed else 1.0)

    def getRobotRelativeSpeeds(self) -> ChassisSpeeds:
        """Returns the robot-relative speeds of the robot.

        :returns: The robot-relative ChassisSpeeds
        """
        # Get the current module states
        moduleStates = (
            self.frontLeft.getState(),
            self.frontRight.getState(),
            self.rearLeft.getState(),
            self.rearRight.getState(),
        )
        # Convert to chassis speeds using the kinematics
        return DriveConstants.kDriveKinematics.toChassisSpeeds(moduleStates)

    def driveRobotRelative(self, speeds: ChassisSpeeds, feedforwards) -> None:
        """Drive the robot given robot-relative ChassisSpeeds.

        :param speeds: The desired robot-relative ChassisSpeeds
        :param feedforwards: The feedforward outputs for each module
        """
        swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds)
        fl, fr, rl, rr = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond
        )
        self.frontLeft.setDesiredState(fl)
        self.frontRight.setDesiredState(fr)
        self.rearLeft.setDesiredState(rl)
        self.rearRight.setDesiredState(rr)

    def shouldFlipPath(self) -> bool:
        """Flips the path based on the alliance color.

        :returns: True if the path should be flipped, False otherwise
        """
        alliance = DriverStation.getAlliance()
        return alliance == DriverStation.Alliance.kRed

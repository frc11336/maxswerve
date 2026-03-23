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
from wpimath.estimator import SwerveDrive4PoseEstimator

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

        try:
            print("Creating MAXSwerveModule - Front Left...")
            # Create MAXSwerveModules
            self.frontLeft = MAXSwerveModule(
                DriveConstants.kFrontLeftDrivingCanId,
                DriveConstants.kFrontLeftTurningCanId,
                DriveConstants.kFrontLeftChassisAngularOffset,
            )

            print("Creating MAXSwerveModule - Front Right...")
            self.frontRight = MAXSwerveModule(
                DriveConstants.kFrontRightDrivingCanId,
                DriveConstants.kFrontRightTurningCanId,
                DriveConstants.kFrontRightChassisAngularOffset,
            )

            print("Creating MAXSwerveModule - Rear Left...")
            self.rearLeft = MAXSwerveModule(
                DriveConstants.kRearLeftDrivingCanId,
                DriveConstants.kRearLeftTurningCanId,
                DriveConstants.kBackLeftChassisAngularOffset,
            )

            print("Creating MAXSwerveModule - Rear Right...")
            self.rearRight = MAXSwerveModule(
                DriveConstants.kRearRightDrivingCanId,
                DriveConstants.kRearRightTurningCanId,
                DriveConstants.kBackRightChassisAngularOffset,
            )

            print("Loading RobotConfig from GUI settings...")
            config = RobotConfig.fromGUISettings()

            # The gyro sensor
            try:
                self.gyro = navx.AHRS.create_spi()
                print("NavX gyro initialized successfully")
            except Exception as e:
                print(f"WARNING: Failed to initialize NavX gyro: {e}")
                # Create a dummy gyro that returns 0 for testing
                self.gyro = None

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
            # Using SwerveDrive4PoseEstimator so vision measurements can be fused in
            print("Initializing SwerveDrive4PoseEstimator...")
            self.odometry = SwerveDrive4PoseEstimator(
                DriveConstants.kDriveKinematics,
                Rotation2d.fromDegrees(-self.gyro.getAngle()) if self.gyro is not None else Rotation2d(),
                (
                    self.frontLeft.getPosition(),
                    self.frontRight.getPosition(),
                    self.rearLeft.getPosition(),
                    self.rearRight.getPosition(),
                ),
                Pose2d(3.66, 4.04, Rotation2d.fromDegrees(0)),
            )

            # Configure the AutoBuilder last
            print("Configuring AutoBuilder...")
            AutoBuilder.configure(
                self.getPose, # Robot pose supplier
                self.resetOdometry, # Method to reset odometry (will be called if your auto has a starting pose)
                self.getRobotRelativeSpeeds, # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                self.driveRobotRelative, # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also outputs individual module feedforwards
                PPHolonomicDriveController(
                    PIDConstants(4.7, 0.0, 1),  # Translation PID
                    PIDConstants(9.5, 0.0, 0.0)   # Rotation PID
                ), # PPHolonomicDriveController for swerve drives
                config, # The robot configuration
                self.shouldFlipPath, # Supplier to control path flipping based on alliance color
                self # Reference to this subsystem to set requirements
            )

            # Create field visualization
            print("Creating Field2d visualization...")
            from wpilib import Field2d
            self.field = Field2d()
            SmartDashboard.putData("Field", self.field)
            
            # For testing max speed
            self.testMode = False
            
            # Cache gyro angle to reduce NavX reads (can be slow/blocking)
            self.cached_gyro_angle = 0.0
            self.gyro_read_counter = 0
            
            # Cache module positions to reduce CAN reads (8 reads per frame = 400/sec!)
            # Store both the position data and rotation data
            self.cached_module_positions = [
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.rearLeft.getPosition(),
                self.rearRight.getPosition(),
            ]
            self.cached_gyro_rotation = Rotation2d()
            self.odometry_update_counter = 0

            # Telemetry counter for low-rate dashboard publishing
            self.telemetry_counter = 0
            # Track yaw at last zero to compute drift over time
            self._yaw_at_last_zero = 0.0
            self._time_at_last_zero = wpilib.Timer.getFPGATimestamp()
            
            print("DriveSubsystem initialization complete!")
        except Exception as e:
            print(f"CRITICAL ERROR initializing DriveSubsystem (odometry/AutoBuilder): {e}")
            import traceback
            traceback.print_exc()
            raise

        

    def periodic(self) -> None:
        # Update the odometry in the periodic block
        # Cache gyro rotation AND module positions to avoid excessive CAN traffic
        
        # Update cached gyro rotation every 3 frames (instead of every frame)
        self.gyro_read_counter += 1
        if self.gyro_read_counter >= 3:
            if self.gyro is not None:
                self.cached_gyro_angle = -self.gyro.getAngle()
                self.cached_gyro_rotation = Rotation2d.fromDegrees(-self.gyro.getAngle())
            else:
                self.cached_gyro_rotation = Rotation2d(0)
            self.gyro_read_counter = 0
        
        # Update cached module positions every 5 frames (CRITICAL: reduces CAN reads from 8/frame to <2/frame)
        # This prevents the driver station communication dropout caused by excessive CAN traffic
        self.odometry_update_counter += 1
        if self.odometry_update_counter >= 5:
            self.cached_module_positions = [
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.rearLeft.getPosition(),
                self.rearRight.getPosition(),
            ]
            self.odometry_update_counter = 0
        
        # Update odometry using cached values (not blocking CAN reads)
        self.odometry.update(
            self.cached_gyro_rotation,
            (
                self.cached_module_positions[0],
                self.cached_module_positions[1],
                self.cached_module_positions[2],
                self.cached_module_positions[3],
            )
        )

        # --- Yaw drift diagnostics (published to Elastic Dashboard at ~5 Hz) ---
        # Published every 10 frames to avoid excessive SmartDashboard traffic
        self.telemetry_counter += 1
        if self.telemetry_counter >= 10:
            self.telemetry_counter = 0
            if self.gyro is not None:
                raw_yaw      = -self.gyro.getAngle()         # cumulative, unbounded degrees (negated)
                yaw_360      = raw_yaw % 360                 # 0-360
                yaw_180      = float(self.gyro.getYaw())     # direct ±180° from NavX AHRS
                turn_rate    = self.gyro.getRate()           # deg/s
                is_cal       = self.gyro.isCalibrating()
                is_connected = self.gyro.isConnected()

                now = wpilib.Timer.getFPGATimestamp()
                elapsed = now - self._time_at_last_zero
                drift    = raw_yaw - self._yaw_at_last_zero  # total drift since last zero

                SmartDashboard.putNumber("Gyro RawYaw deg",         raw_yaw)
                SmartDashboard.putNumber("Gyro Yaw 0to360",          yaw_360)
                SmartDashboard.putNumber("Gyro Yaw 180",             yaw_180)
                SmartDashboard.putNumber("Gyro TurnRate degps",      turn_rate)
                SmartDashboard.putNumber("Gyro DriftSinceZero deg",  drift)
                SmartDashboard.putNumber("Gyro SecondsSinceZero",    elapsed)
                SmartDashboard.putBoolean("Gyro Was Calibrating",      is_cal)
                SmartDashboard.putBoolean("Gyro IsConnected",        is_connected)

                # Estimated pose from the pose estimator (includes vision corrections)
                pose = self.odometry.getEstimatedPosition()
                SmartDashboard.putNumber("Odometry X m",             pose.X())
                SmartDashboard.putNumber("Odometry Y m",             pose.Y())
                SmartDashboard.putNumber("Odometry Heading deg",     pose.rotation().degrees())
        
        
        # Update field visualization - DISABLED due to blocking issues
        # These SmartDashboard calls can block the main thread
        # TODO: Consider using asynchronous updates or a separate thread
        """
        # Only update SmartDashboard ~10 times per second instead of 50 times per second
        frame_count = int(wpilib.Timer.getFPGATimestamp() * 10) % 5
        if frame_count == 0:
            self.field.setRobotPose(self.odometry.getPose())
            
            # Publish pose to SmartDashboard for visualization
            pose = self.odometry.getPose()
            SmartDashboard.putNumber("Robot X", pose.X())
            SmartDashboard.putNumber("Robot Y", pose.Y())
            SmartDashboard.putNumber("Robot Angle", pose.rotation().degrees())
            
            # Publish actual measured speeds for testing
            speeds = self.getRobotRelativeSpeeds()
            SmartDashboard.putNumber("Measured Speed X", speeds.vx)
            SmartDashboard.putNumber("Measured Speed Y", speeds.vy)
            SmartDashboard.putNumber("Measured Speed Magnitude", math.sqrt(speeds.vx**2 + speeds.vy**2))
        """
    
    def getPose(self) -> Pose2d:
        """Returns the currently-estimated pose of the robot.

        :returns: The pose.
        """
        return self.odometry.getEstimatedPosition()

    def resetOdometry(self, pose: Pose2d) -> None:
        """Resets the odometry to the specified pose.

        :param pose: The pose to which to set the odometry.

        """
        if self.gyro is not None:
            angle = Rotation2d.fromDegrees(-self.gyro.getAngle())
        else:
            angle = Rotation2d(0)
            
        self.odometry.resetPosition(
            angle,
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
                Rotation2d.fromDegrees(self.cached_gyro_angle),
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

    def addVisionMeasurement(self, pose: Pose2d, timestamp: float) -> None:
        """Feed a vision pose estimate into the pose estimator.

        :param pose:      The field-relative pose measured by the camera.
        :param timestamp: The FPGA timestamp (seconds) at which the image was captured.
                          Use the value returned by the Limelight helper, NOT getFPGATimestamp().
        """
        try:
            self.odometry.addVisionMeasurement(pose, timestamp)
        except Exception as e:
            print(f"[DRIVE] addVisionMeasurement error: {e}")

    def zeroHeading(self) -> None:
        """Zeroes the heading of the robot and resets the drift baseline."""
        if self.gyro:
            self.gyro.reset()
            self._yaw_at_last_zero = 0.0
            self._time_at_last_zero = wpilib.Timer.getFPGATimestamp()

    def getHeading(self) -> float:
        """Returns the heading of the robot in WPILib convention (CCW-positive).

        :returns: the robot's heading in degrees, from -180 to 180
        """
        if self.gyro:
            return -Rotation2d.fromDegrees(self.gyro.getAngle()).degrees()
        return 0.0

    def getRawYaw(self) -> float:
        """Returns the raw NavX yaw in the NavX's native convention (CW-positive).

        Use this when feeding yaw to the Limelight for MegaTag2 — Limelight expects
        the same CW-positive convention as the NavX, NOT the WPILib CCW-positive convention.

        :returns: yaw in degrees, -180 to 180
        """
        if self.gyro:
            return float(self.gyro.getYaw())
        return 0.0

    def getTurnRate(self) -> float:
        """Returns the turn rate of the robot.

        :returns: The turn rate of the robot, in degrees per second
        """
        if self.gyro:
            return self.gyro.getRate() * (-1.0 if DriveConstants.kGyroReversed else 1.0)
        return 0.0

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

    def testMaxSpeedForward(self) -> None:
        """Drives forward at maximum speed for testing."""
        self.drive(1.0, 0.0, 0.0, False, False)

    def testMaxSpeedStrafe(self) -> None:
        """Strafes left at maximum speed for testing."""
        self.drive(0.0, 1.0, 0.0, False, False)

    def testStop(self) -> None:
        """Stops all drive motors."""
        self.drive(0.0, 0.0, 0.0, False, False)

    def rotate(self, rotSpeed) -> None:
        """
        Rotate the robot in place, without moving laterally (for example, for aiming)
        :param speed: rotation speed 
        """
        self.drive(0, 0, rotSpeed, False, False)
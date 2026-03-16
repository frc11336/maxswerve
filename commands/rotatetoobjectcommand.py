import commands2
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.limelight import LimelightCamera
import math
import wpilib


class RotateToObjectCommand(commands2.Command):
    """Command to rotate the robot to face an April tag detected by Limelight"""

    def __init__(self, drive: DriveSubsystem, camera: LimelightCamera):
        super().__init__()
        self.drive = drive
        self.camera = camera
        
        # Add drive subsystem requirement
        self.addRequirements(drive)
        
        # Tolerance for alignment (degrees)
        self.ALIGNMENT_TOLERANCE = 3.0
        self.MAX_ROTATION_SPEED = 0.5
        self.ROTATION_GAIN = 0.01  # How aggressively to turn
        
        # Cache the last x_offset to reduce NetworkTables calls
        self.last_x_offset = 0
        self.frame_counter = 0
        
        # Timeout to prevent command from running forever
        self.start_time = 0
        self.COMMAND_TIMEOUT = 5.0  # Maximum 5 seconds to align

    def initialize(self):
        """Called when the command starts"""
        self.start_time = wpilib.Timer.getFPGATimestamp()
        self.frame_counter = 0
        self.last_x_offset = 0
        print("RotateToObjectCommand started")

    def execute(self):
        """Called repeatedly while the command is running"""
        try:
            # Only read from Limelight every 2 frames (25Hz instead of 50Hz)
            # to reduce NetworkTables traffic that could block the thread
            if self.frame_counter % 2 == 0:
                self.last_x_offset = self.camera.getX()
            
            self.frame_counter += 1
            x_offset = self.last_x_offset
            
            # If x_offset is 0, that usually means we're aligned (centered)
            # Don't return early - let isFinished() handle the logic
            
            # Calculate rotation speed based on offset
            # Positive offset means object is to the right, so we turn right (positive rotation)
            rotation_speed = -self.ROTATION_GAIN * x_offset
            
            # Limit the rotation speed
            rotation_speed = max(-self.MAX_ROTATION_SPEED, min(self.MAX_ROTATION_SPEED, rotation_speed))
            
            # Drive only with rotation, no translation
            self.drive.drive(0, 0, rotation_speed, True, False)
        except Exception as e:
            print(f"Error in RotateToObjectCommand.execute(): {e}")
            self.drive.drive(0, 0, 0, False, False)

    def end(self, interrupted: bool):
        """Called when the command ends"""
        # Stop the robot
        self.drive.drive(0, 0, 0, False, False)
        if interrupted:
            print("RotateToObjectCommand interrupted - joystick control restored")
        else:
            print("RotateToObjectCommand finished - aligned and returning control to joystick")

    def isFinished(self) -> bool:
        """Returns true when the command should end"""
        try:
            # Use cached value to avoid NetworkTables read every frame
            x_offset = self.last_x_offset
            
            # Check if we've exceeded timeout
            elapsed = wpilib.Timer.getFPGATimestamp() - self.start_time
            if elapsed > self.COMMAND_TIMEOUT:
                print(f"RotateToObjectCommand timeout after {elapsed:.1f}s")
                return True
            
            # Command is finished when the offset is within tolerance
            is_aligned = abs(x_offset) < self.ALIGNMENT_TOLERANCE
            
            if is_aligned:
                print(f"Robot aligned! Offset: {x_offset:.2f} degrees - isFinished() returning True")
            
            return is_aligned
        except Exception as e:
            print(f"Error in RotateToObjectCommand.isFinished(): {e}")
            return True  # End command on error

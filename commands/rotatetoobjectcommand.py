import commands2
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.limelight import LimelightCamera
import math


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

    def initialize(self):
        """Called when the command starts"""
        print("RotateToObjectCommand started")

    def execute(self):
        """Called repeatedly while the command is running"""
        try:
            # Get the horizontal offset from Limelight
            x_offset = self.camera.getX()
            
            # If x_offset is 0 and we have no target, stop
            if x_offset == 0:
                self.drive.drive(0, 0, 0, False, False)
                return
            
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
            print("RotateToObjectCommand interrupted")
        else:
            print("RotateToObjectCommand finished")

    def isFinished(self) -> bool:
        """Returns true when the command should end"""
        try:
            x_offset = self.camera.getX()
            
            # Command is finished when the offset is within tolerance
            is_aligned = abs(x_offset) < self.ALIGNMENT_TOLERANCE
            
            if is_aligned:
                print(f"Robot aligned! Offset: {x_offset}")
            
            return is_aligned
        except Exception as e:
            print(f"Error in RotateToObjectCommand.isFinished(): {e}")
            return True  # End command on error

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
        self.ROTATION_GAIN = 0.05  # How aggressively to turn
        
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
        self.debug_print_counter = 0
        print("[ROTATE] ========== COMMAND STARTED ==========")
        print(f"[ROTATE] Alignment tolerance: {self.ALIGNMENT_TOLERANCE}°")
        print(f"[ROTATE] Rotation gain: {self.ROTATION_GAIN}")
        print(f"[ROTATE] Camera heartbeating: {self.camera.heartbeating}")
        print(f"[ROTATE] Initial x_offset: {self.camera.getX()}")

    def execute(self):
        """Called repeatedly while the command is running"""
        try:
            # Only read cached value every 3 frames (every 60ms at 50Hz = ~16Hz)
            # The Limelight subsystem already caches values every 3 frames
            # So reading every 3 frames here gives us 16Hz updates
            if self.frame_counter % 3 == 0:
                self.last_x_offset = self.camera.getX()
            
            self.frame_counter += 1
            x_offset = self.last_x_offset
            
            # Debug output every 10 frames (~200ms)
            if self.frame_counter % 10 == 0:
                calculated_speed = self.ROTATION_GAIN * x_offset
                print(f"[ROTATE] Frame {self.frame_counter}: x_offset={x_offset:.3f}°, rotation_speed={calculated_speed:.4f}, heartbeating={self.camera.heartbeating}")
            
            # Calculate rotation speed based on offset
            # Positive offset means object is to the right, so we turn right (positive rotation)
            rotation_speed = -self.ROTATION_GAIN * x_offset
            
            # Limit the rotation speed
            rotation_speed = max(-self.MAX_ROTATION_SPEED, min(self.MAX_ROTATION_SPEED, rotation_speed))
            
            # Drive only with rotation, no translation
            self.drive.drive(0, 0, rotation_speed, True, False)
        except Exception as e:
            print(f"[ROTATE] Error in execute(): {e}")
            self.drive.drive(0, 0, 0, False, False)

    def end(self, interrupted: bool):
        """Called when the command ends"""
        # Stop the robot immediately - critical for safety
        self.drive.drive(0, 0, 0, False, False)
        
        # Log completion (minimal print to avoid blocking)
        if interrupted:
            print("[ROTATE] Command interrupted - joystick control restored")
        else:
            print("[ROTATE] Command finished - robot aligned")

    def isFinished(self) -> bool:
        """Returns true when the command should end"""
        try:
            # Use cached value to avoid NetworkTables read every frame
            x_offset = self.last_x_offset
            
            # Check if we've exceeded timeout
            elapsed = wpilib.Timer.getFPGATimestamp() - self.start_time
            if elapsed > self.COMMAND_TIMEOUT:
                print(f"[ROTATE] ========== TIMEOUT after {elapsed:.1f}s ==========")
                print(f"[ROTATE] Final x_offset: {x_offset:.3f}°")
                print(f"[ROTATE] Final frame_counter: {self.frame_counter}")
                print(f"[ROTATE] Alignment check: abs({x_offset:.3f}) < {self.ALIGNMENT_TOLERANCE} = {abs(x_offset) < self.ALIGNMENT_TOLERANCE}")
                print(f"[ROTATE] Frame check: {self.frame_counter} > 5 = {self.frame_counter > 5}")
                print(f"[ROTATE] Overall aligned: {abs(x_offset) < self.ALIGNMENT_TOLERANCE and self.frame_counter > 5}")
                return True
            
            # Command is finished when the offset is within tolerance
            # Wait a few frames (frame_counter > 5) to ensure stable reading
            is_aligned = abs(x_offset) < self.ALIGNMENT_TOLERANCE and self.frame_counter > 5
            
            if is_aligned:
                print(f"[ROTATE] ========== ALIGNED at {x_offset:.3f}° ==========")
                return True
            
            return False
        except Exception as e:
            print(f"[ROTATE] Error in isFinished(): {e}")
            return True  # End command on error

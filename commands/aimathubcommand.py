import commands2
import wpilib
import math
from wpimath.geometry import Translation2d
from subsystems.drivesubsystem import DriveSubsystem


# Field hub positions (metres, WPIBlue origin)
RED_HUB  = Translation2d(11.91, 4.03)
BLUE_HUB = Translation2d(4.63,  4.03)


def nearest_hub(robot_pose) -> Translation2d:
    """Return whichever hub is closer to the robot's current pose."""
    robot_t = robot_pose.translation()
    d_red  = robot_t.distance(RED_HUB)
    d_blue = robot_t.distance(BLUE_HUB)
    return RED_HUB if d_red < d_blue else BLUE_HUB


def distance_to_nearest_hub(robot_pose) -> float:
    """Return the straight-line distance (metres) to the nearest hub."""
    return robot_pose.translation().distance(nearest_hub(robot_pose))


class AimAtHubCommand(commands2.Command):
    """Rotate the robot so its front faces the nearest scoring hub,
    using odometry (no camera required).

    The angle to the hub is computed from the robot's field position every
    frame, then the drivetrain is rotated to close the heading error.
    """

    ALIGNMENT_TOLERANCE_DEG = 2.0   # degrees — finish when within this
    MAX_ROTATION_SPEED      = 0.45  # 0-1 motor output
    ROTATION_GAIN           = 0.025 # output per degree of error
    COMMAND_TIMEOUT         = 5.0   # seconds

    def __init__(self, drive):
        super().__init__()
        self.drive = drive
        self.addRequirements(drive)

        self.start_time   = 0.0
        self.frame_counter = 0

    # ------------------------------------------------------------------
    def initialize(self, drive):
        self.drive = drive
        self.start_time    = wpilib.Timer.getFPGATimestamp()
        self.frame_counter = 0
        pose = self.drive.getPose()
        hub  = nearest_hub(pose)
        dist = distance_to_nearest_hub(pose)
        print(f"[AIM] Started — hub=({hub.X():.2f},{hub.Y():.2f}), dist={dist:.2f}m")

    # ------------------------------------------------------------------
    def execute(self, drive):
        self.drive = drive
        try:
            self.frame_counter += 1
            pose = self.drive.getPose()
            hub  = nearest_hub(pose)

            # Angle FROM robot TO hub in field coordinates (CCW-positive, degrees)
            dx = hub.X() - pose.X()
            dy = hub.Y() - pose.Y()
            target_heading_deg = math.degrees(math.atan2(dy, dx))

            # Current robot heading (WPILib CCW-positive, -180..180)
            current_heading_deg = pose.rotation().degrees()

            # Shortest-path heading error
            error = target_heading_deg - current_heading_deg
            # Wrap to -180..180
            error = (error + 180) % 360 - 180

            rotation_speed = -self.ROTATION_GAIN * error
            rotation_speed = max(-self.MAX_ROTATION_SPEED,
                                 min(self.MAX_ROTATION_SPEED, rotation_speed))

            if self.frame_counter % 10 == 0:
                print(f"[AIM] target={target_heading_deg:.1f}° "
                      f"current={current_heading_deg:.1f}° "
                      f"error={error:.1f}° speed={rotation_speed:.3f}")

            self.drive.drive(0, 0, rotation_speed, True, False)

        except Exception as e:
            print(f"[AIM] Error in execute(): {e}")
            self.drive.drive(0, 0, 0, False, False)

    # ------------------------------------------------------------------
    def end(self, interrupted: bool):
        self.drive.drive(0, 0, 0, False, False)
        status = "interrupted" if interrupted else "aligned to hub"
        print(f"[AIM] Command ended — {status}")

    # ------------------------------------------------------------------
    def isFinished(self) -> bool:
        try:
            elapsed = wpilib.Timer.getFPGATimestamp() - self.start_time
            if elapsed > self.COMMAND_TIMEOUT:
                print(f"[AIM] Timeout after {elapsed:.1f}s")
                return True

            if self.frame_counter < 5:
                return False

            pose = self.drive.getPose()
            hub  = nearest_hub(pose)
            dx   = hub.X() - pose.X()
            dy   = hub.Y() - pose.Y()
            target_heading_deg  = math.degrees(math.atan2(dy, dx))
            current_heading_deg = pose.rotation().degrees()
            error = (target_heading_deg - current_heading_deg + 180) % 360 - 180

            if abs(error) < self.ALIGNMENT_TOLERANCE_DEG:
                print(f"[AIM] Aligned — error={error:.2f}°")
                return True

            return False

        except Exception as e:
            print(f"[AIM] Error in isFinished(): {e}")
            return True
    def aim(self, drive):
        self.drive
        self.initialize(self.drive)
        self.execute(self.drive)
        self.isFinished()
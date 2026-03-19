#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from wpilib import Timer
from commands2 import Subsystem
from ntcore import NetworkTableInstance
from wpimath.geometry import Pose2d, Rotation2d


class LimelightCamera(Subsystem):
    def __init__(self, cameraName: str, drive=None) -> None:
        """
        :param cameraName: Name of the Limelight table (e.g. "limelight").
        :param drive:      Optional DriveSubsystem reference.  When provided,
                           the subsystem will (a) push robot yaw to the Limelight
                           every frame so MegaTag2 works correctly, and (b) feed
                           vision pose estimates back into the drive pose estimator.
        """
        super().__init__()

        self.cameraName = _fix_name(cameraName)
        self.drive = drive  # DriveSubsystem reference (may be None)

        instance = NetworkTableInstance.getDefault()
        self.table = instance.getTable(self.cameraName)
        self._path = self.table.getPath()

        self.pipelineIndexRequest = self.table.getDoubleTopic("pipeline").publish()
        self.pipelineIndex = self.table.getDoubleTopic("getpipe").getEntry(-1)

        self.ledMode = self.table.getIntegerTopic("ledMode").getEntry(-1)
        self.camMode = self.table.getIntegerTopic("camMode").getEntry(-1)
        self.tx = self.table.getDoubleTopic("tx").getEntry(0.0)
        self.ty = self.table.getDoubleTopic("ty").getEntry(0.0)
        self.ta = self.table.getDoubleTopic("ta").getEntry(0.0)
        self.hb = self.table.getIntegerTopic("hb").getEntry(0)

        # MegaTag2: publish robot orientation [yaw, yawRate, pitch, pitchRate, roll, rollRate]
        # Limelight reads this every frame to improve pose estimation
        self.robot_orientation_pub = self.table.getDoubleArrayTopic("robot_orientation_set").publish()

        # MegaTag2 pose (field-relative, WPIBlue origin) – [x, y, z, roll, pitch, yaw, latency, tagCount, ...]
        self.botpose_wpiblue = self.table.getDoubleArrayTopic("botpose_orb_wpiblue").getEntry([])

        self.lastHeartbeat = 0
        self.lastHeartbeatTime = 0
        self.heartbeating = False

        # Cache NetworkTables values to reduce blocking I/O
        self.cached_tx = 0.0
        self.cached_ty = 0.0
        self.cached_ta = 0.0
        self.cached_hb = 0
        self.nt_read_counter = 0
        self.yaw_write_counter = 0

    def setPipeline(self, index: int):
        self.pipelineIndexRequest.set(float(index))

    def getPipeline(self) -> int:
        return int(self.pipelineIndex.get(-1))

    def getA(self) -> float:
        return self.cached_ta

    def getX(self) -> float:
        return self.cached_tx

    def getY(self) -> float:
        return self.cached_ty

    def getHB(self) -> float:
        return self.cached_hb

    def hasDetection(self):
        if self.getX() != 0.0 and self.heartbeating:
            return True

    def getSecondsSinceLastHeartbeat(self) -> float:
        return Timer.getFPGATimestamp() - self.lastHeartbeatTime

    def periodic(self) -> None:
        now = Timer.getFPGATimestamp()
        try:
            # --- Push robot yaw to Limelight every frame for MegaTag2 ---
            # MegaTag2 needs the robot's current yaw so it can resolve pose ambiguity.
            # We write every frame (cheap NT publish) rather than caching, because
            # a stale yaw degrades MegaTag2 accuracy more than the small NT overhead.
            if self.drive is not None:
                yaw_deg = self.drive.getHeading()  # degrees, -180..180
                # Format: [yaw, yawRate, pitch, pitchRate, roll, rollRate]
                self.robot_orientation_pub.set([yaw_deg, 0.0, 0.0, 0.0, 0.0, 0.0])

            # --- Read from NetworkTables every 3 frames to reduce blocking I/O ---
            self.nt_read_counter += 1
            if self.nt_read_counter >= 3:
                self.cached_tx = self.tx.get()
                self.cached_ty = self.ty.get()
                self.cached_ta = self.ta.get()
                self.cached_hb = self.hb.get()
                self.nt_read_counter = 0

                # --- Feed MegaTag2 vision pose into the drive pose estimator ---
                if self.drive is not None:
                    pose_data = self.botpose_wpiblue.get([])
                    # botpose_orb_wpiblue has at least 7 elements:
                    #   [x, y, z, roll, pitch, yaw, total_latency, ...]
                    # Only trust the measurement when at least one tag is visible (ta > 0)
                    if len(pose_data) >= 7 and self.cached_ta > 0:
                        x, y, _z, _roll, _pitch, yaw_deg_pose, latency_ms = pose_data[:7]
                        # Convert capture timestamp: now minus the pipeline latency
                        capture_time = now - (latency_ms / 1000.0)
                        vision_pose = Pose2d(x, y, Rotation2d.fromDegrees(yaw_deg_pose))
                        self.drive.addVisionMeasurement(vision_pose, capture_time)

            # --- Heartbeat monitoring ---
            heartbeat = self.cached_hb
            if heartbeat != self.lastHeartbeat:
                self.lastHeartbeat = heartbeat
                self.lastHeartbeatTime = now
            heartbeating = now < self.lastHeartbeatTime + 5
            if heartbeating != self.heartbeating:
                if int(now * 2) % 2 == 0:
                    print(f"Camera {self.cameraName} is " + ("UPDATING" if heartbeating else "NO LONGER UPDATING") + f" (hb={heartbeat})")
            self.heartbeating = heartbeating
        except Exception as e:
            pass


def _fix_name(name: str):
    if not name:
        name = "limelight"
    return name

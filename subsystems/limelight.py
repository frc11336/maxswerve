#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from wpilib import Timer
from commands2 import Subsystem
from ntcore import NetworkTableInstance


class LimelightCamera(Subsystem):
    def __init__(self, cameraName: str) -> None:
        super().__init__()

        self.cameraName = _fix_name(cameraName)

        instance = NetworkTableInstance.getDefault()
        self.table = instance.getTable(self.cameraName)
        self._path = self.table.getPath()

        self.pipelineIndexRequest = self.table.getDoubleTopic("pipeline").publish()
        self.pipelineIndex = self.table.getDoubleTopic("getpipe").getEntry(-1)
        # "cl" and "tl" are additional latencies in milliseconds

        self.ledMode = self.table.getIntegerTopic("ledMode").getEntry(-1)
        self.camMode = self.table.getIntegerTopic("camMode").getEntry(-1)
        self.tx = self.table.getDoubleTopic("tx").getEntry(0.0)
        self.ty = self.table.getDoubleTopic("ty").getEntry(0.0)
        self.ta = self.table.getDoubleTopic("ta").getEntry(0.0)
        self.hb = self.table.getIntegerTopic("hb").getEntry(0)
        self.lastHeartbeat = 0
        self.lastHeartbeatTime = 0
        self.heartbeating = False
        
        # Cache NetworkTables values to reduce blocking I/O
        self.cached_tx = 0.0
        self.cached_ty = 0.0
        self.cached_ta = 0.0
        self.cached_hb = 0
        self.nt_read_counter = 0

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
            # Only read from NetworkTables every 3 frames to reduce blocking I/O
            # This caches all camera values: tx, ty, ta, hb
            self.nt_read_counter += 1
            if self.nt_read_counter >= 3:
                self.cached_tx = self.tx.get()
                self.cached_ty = self.ty.get()
                self.cached_ta = self.ta.get()
                self.cached_hb = self.hb.get()
                # Debug: Show what we're reading from Limelight (uncomment to debug)
                #print(f"[LIMELIGHT] Updated cache: tx={self.cached_tx:.2f}, ty={self.cached_ty:.2f}, ta={self.cached_ta:.2f}, hb={self.cached_hb}")
                self.nt_read_counter = 0
            
            # Use a try/except to handle network timeout gracefully
            heartbeat = self.cached_hb
            if heartbeat != self.lastHeartbeat:
                self.lastHeartbeat = heartbeat
                self.lastHeartbeatTime = now
            heartbeating = now < self.lastHeartbeatTime + 5  # no heartbeat for 5s => stale camera
            if heartbeating != self.heartbeating:
                # Only print occasionally to avoid spamming console
                if int(now * 2) % 2 == 0:  # Print once every 2 seconds
                    print(f"Camera {self.cameraName} is " + ("UPDATING" if heartbeating else "NO LONGER UPDATING") + f" (hb={heartbeat})")
            self.heartbeating = heartbeating
        except Exception as e:
            # Silently handle network errors to prevent blocking
            pass


def _fix_name(name: str):
    if not name:
        name = "limelight"
    return name

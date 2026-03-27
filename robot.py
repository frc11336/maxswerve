#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import wpilib

from robotcontainer import RobotContainer


class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self):
        try:
            print("=" * 60)
            print("ROBOT INITIALIZATION STARTED")
            print("=" * 60)
            # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
            # autonomous chooser on the dashboard.
            self.container = RobotContainer()
            self.autonomousCommand = None
            print("=" * 60)
            print("ROBOT INITIALIZATION COMPLETE")
            print("=" * 60)
        except Exception as e:
            print("=" * 60)
            print(f"CRITICAL ERROR DURING ROBOT INITIALIZATION: {e}")
            print("=" * 60)
            import traceback
            traceback.print_exc()
            raise
        

    def robotPeriodic(self) -> None:
        # Keep distance-to-hub current every loop cycle (~50Hz)
        # This means self.container.distance is always up to date for shooting
        try:
            self.container.updateDistance()
        except Exception:
            pass

    def autonomousInit(self) -> None:
        self.autonomousCommand = self.container.autoChooser.getSelected()

        if self.autonomousCommand:
            # Reset odometry to the starting pose defined in the selected auto
            # PathPlanner will also call resetOdometry() via AutoBuilder, but
            # doing it here ensures the pose estimator is correct before the
            # first command runs
            starting_pose = self.autonomousCommand.getStartingPose() if hasattr(self.autonomousCommand, 'getStartingPose') else None
            if starting_pose is not None:
                self.container.robotDrive.resetOdometry(starting_pose)
            self.autonomousCommand.schedule()

    def teleopInit(self) -> None:
        if self.autonomousCommand:
            self.autonomousCommand.cancel()

    def testInit(self) -> None:
        commands2.CommandScheduler.getInstance().cancelAll()


if __name__ == "__main__":
    wpilib.run(MyRobot)

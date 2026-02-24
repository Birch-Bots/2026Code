"""
robot.py — FRC Team 5421 | Swerve Drive
Minimal robot — drivetrain only.

DRIVER CONTROLS (Xbox, Port 0):
  Left  Stick   → Move (field-centric)
  Right Stick X → Rotate
  Left  Bumper  → Slow mode (30%)
  Start         → Zero gyro
  Back          → X-lock wheels
"""

import wpilib
import commands2
from subsystems.drivetrain import Drivetrain
from commands.default_drive import DefaultDrive


class MyRobot(commands2.TimedCommandRobot):

    def robotInit(self):
        wpilib.DataLogManager.start()
        wpilib.DriverStation.silenceJoystickConnectionWarning(True)

        self.drivetrain = Drivetrain()
        self.driver     = wpilib.XboxController(0)

        self.drivetrain.setDefaultCommand(
            DefaultDrive(self.drivetrain, self.driver)
        )

    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()

    def teleopInit(self):
        commands2.CommandScheduler.getInstance().cancelAll()

    def disabledInit(self):
        commands2.CommandScheduler.getInstance().cancelAll()


if __name__ == "__main__":
    wpilib.run(MyRobot)

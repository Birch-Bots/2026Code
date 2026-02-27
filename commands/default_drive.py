"""
default_drive.py — FRC Team 5421
Robot-centric teleop swerve drive command.

  Left  Stick     → Translation (vx / vy)
  Right Stick X   → Rotation
  Left  Bumper    → Slow mode (30%)
  Start Button    → Zero gyro heading
  Back  Button    → X-lock wheels
"""
vx = 0
vy = 0
omega = 0
import math
import wpilib
import commands2
from wpimath.filter import SlewRateLimiter
from wpimath.kinematics import ChassisSpeeds
from subsystems.drivetrain import Drivetrain
from constants.constants import SwerveConstants, OI


def _deadband(value: float, db: float) -> float:
    if abs(value) < db:
        return 0.0
    return (value - math.copysign(db, value)) / (1.0 - db)


class DefaultDrive(commands2.Command):

    def __init__(self, drivetrain: Drivetrain, controller: wpilib.XboxController):
        super().__init__()
        self.dt   = drivetrain
        self.ctrl = controller
        self.addRequirements(drivetrain)

        self._vx_lim    = SlewRateLimiter(8)
        self._vy_lim    = SlewRateLimiter(8)
        self._omega_lim = SlewRateLimiter(8)



    def execute(self):
        if self.ctrl.getBackButton():
            self.dt.lock_wheels()
            return

        if self.ctrl.getStartButtonPressed():
            self.dt.zero_heading()

        db = OI.DRIVER_DEADBAND
        vx    = _deadband(-self.ctrl.getLeftY(),  db)
        vy    = _deadband(-self.ctrl.getLeftX(),  db)
        omega = _deadband(-self.ctrl.getRightX(), db)

        vx    = math.copysign(vx ** 2, vx)
        vy    = math.copysign(vy ** 2, vy)
        omega = math.copysign(omega ** 2, omega)

        if vx == 0.0 and vy == 0.0 and omega == 0.0:
            self.dt.stop()
            return

        slow = 0.30 if self.ctrl.getLeftBumper() else 1.0
        max_v = SwerveConstants.MAX_DRIVE_SPEED_MPS
        max_w = SwerveConstants.MAX_ANGULAR_SPEED_RPS

        speeds = ChassisSpeeds(
            self._vx_lim.calculate(vx    * max_v * slow),
            self._vy_lim.calculate(vy    * max_v * slow),
            self._omega_lim.calculate(omega * max_w * slow),
        )
        self.dt.drive_robot_relative(speeds)

    def end(self, interrupted):
        self.dt.stop()

    def isFinished(self):
        return False
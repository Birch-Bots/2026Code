"""
drivetrain.py — FRC Team 5421
Field-centric swerve drivetrain.
  - 4x SwerveModule (SparkMAX NEO + CANcoder)
  - NavX2-MXP over SPI
  - SwerveDrive4PoseEstimator for odometry
"""

import wpilib
import commands2
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState, SwerveDrive4Kinematics
from wpimath.estimator import SwerveDrive4PoseEstimator
from navx import AHRS

from subsystems.swerve_module import SwerveModule
from constants.constants import CANIDs, SwerveConstants


class Drivetrain(commands2.Subsystem):

    def __init__(self):
        super().__init__()
        self.setName("Drivetrain")

        # ── Swerve Modules ────────────────────────────────────
        # drive_inverted: flip if your module drives backwards
        # steer_inverted: flip if module steers the wrong direction
        self.front_left = SwerveModule(
            CANIDs.FRONT_LEFT_DRIVE,  CANIDs.FRONT_LEFT_STEER,
            CANIDs.FRONT_LEFT_CANCODER, SwerveConstants.FRONT_LEFT_OFFSET,
            drive_inverted=True, steer_inverted=True,
            module_name="FrontLeft",
        )
        self.front_right = SwerveModule(
            CANIDs.FRONT_RIGHT_DRIVE, CANIDs.FRONT_RIGHT_STEER,
            CANIDs.FRONT_RIGHT_CANCODER, SwerveConstants.FRONT_RIGHT_OFFSET,
            drive_inverted=False, steer_inverted=True,
            module_name="FrontRight",
        )
        self.back_left = SwerveModule(
            CANIDs.BACK_LEFT_DRIVE,   CANIDs.BACK_LEFT_STEER,
            CANIDs.BACK_LEFT_CANCODER, SwerveConstants.BACK_LEFT_OFFSET,
            drive_inverted=True, steer_inverted=True,
            module_name="BackLeft",
        )
        self.back_right = SwerveModule(
            CANIDs.BACK_RIGHT_DRIVE,  CANIDs.BACK_RIGHT_STEER,
            CANIDs.BACK_RIGHT_CANCODER, SwerveConstants.BACK_RIGHT_OFFSET,
            drive_inverted=False, steer_inverted=True,
            module_name="BackRight",
        )
        self._modules = [
            self.front_left,
            self.front_right,
            self.back_left,
            self.back_right,
        ]

        # ── NavX IMU ─────────────────────────────────────────
        self.gyro = AHRS(AHRS.NavXComType.kMXP_SPI)
        self.zero_heading()

        # ── Pose estimator ───────────────────────────────────
        self._pose_estimator = SwerveDrive4PoseEstimator(
            SwerveConstants.KINEMATICS,
            self._get_gyro_rotation(),
            self._get_module_positions(),
            Pose2d(),
        )

        # ── Field visualization ──────────────────────────────
        self._field = wpilib.Field2d()
        wpilib.SmartDashboard.putData("Field", self._field)

    # ── Periodic ─────────────────────────────────────────────

    def periodic(self):
        self._pose_estimator.update(
            self._get_gyro_rotation(),
            self._get_module_positions(),
        )
        pose = self.get_pose()
        self._field.setRobotPose(pose)

        wpilib.SmartDashboard.putNumber("Heading (deg)", self.get_heading())
        wpilib.SmartDashboard.putNumber("Pose X (m)",    round(pose.x, 2))
        wpilib.SmartDashboard.putNumber("Pose Y (m)",    round(pose.y, 2))

    # ── Drive API ─────────────────────────────────────────────

    def drive_field_centric(
        self,
        vx_mps: float,
        vy_mps: float,
        omega_rps: float,
    ) -> None:
        """
        Field-relative drive.
          vx    : forward speed m/s  (+ = away from your driver station)
          vy    : strafe speed  m/s  (+ = left)
          omega : rotation  rad/s    (+ = counter-clockwise)
        """
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            vx_mps, vy_mps, omega_rps, self._get_gyro_rotation()
        )
        self._set_module_states(speeds)

    def drive_robot_relative(self, speeds: ChassisSpeeds) -> None:
        self._set_module_states(speeds)

    def stop(self) -> None:
        for module in self._modules:
            module.stop()

    def lock_wheels(self) -> None:
        """X-pattern to resist being pushed."""
        angles = [45.0, -45.0, -45.0, 45.0]
        for module, angle in zip(self._modules, angles):
            module.set_desired_state(
                SwerveModuleState(0.0, Rotation2d.fromDegrees(angle))
            )

    # ── Pose ──────────────────────────────────────────────────

    def get_pose(self) -> Pose2d:
        return self._pose_estimator.getEstimatedPosition()

    def reset_pose(self, pose: Pose2d) -> None:
        self._pose_estimator.resetPosition(
            self._get_gyro_rotation(),
            self._get_module_positions(),
            pose,
        )

    # ── Gyro ──────────────────────────────────────────────────

    def zero_heading(self) -> None:
        self.gyro.reset()

    def get_heading(self) -> float:
        """Heading in degrees, CCW positive (WPILib convention)."""
        return -self.gyro.getAngle()

    def _get_gyro_rotation(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.get_heading())

    # ── Helpers ───────────────────────────────────────────────

    def _set_module_states(self, speeds: ChassisSpeeds) -> None:
        states = SwerveConstants.KINEMATICS.toSwerveModuleStates(speeds)
        states = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            states, SwerveConstants.MAX_DRIVE_SPEED_MPS
        )
        for module, state in zip(self._modules, states):
            module.set_desired_state(state)

    def _get_module_positions(self):
        return tuple(m.get_position() for m in self._modules)

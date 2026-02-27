"""
swerve_module.py — FRC Team 5421
Single swerve module:
  - Drive: REV NEO via SparkMAX
  - Steer: REV NEO 550 via SparkMAX
  - Absolute encoder: CANcoder (Phoenix 6)
"""

import math
import rev
import phoenix6
from phoenix6 import hardware as ph6_hw, configs, signals
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from constants.constants import SwerveConstants


class SwerveModule:

    def __init__(
        self,
        drive_id: int,
        steer_id: int,
        cancoder_id: int,
        cancoder_offset_degrees: float,
        drive_inverted: bool = False,
        steer_inverted: bool = True,
        module_name: str = "Module",
    ):
        self.name = module_name

        # ── Drive Motor ───────────────────────────────────────
        self.drive_motor = rev.SparkMax(drive_id, rev.SparkMax.MotorType.kBrushless)
        drive_cfg = self._build_drive_config(drive_inverted)
        self.drive_motor.configure(
            drive_cfg,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )
        self.drive_encoder = self.drive_motor.getEncoder()
        self.drive_pid     = self.drive_motor.getClosedLoopController()

        # ── Steer Motor ───────────────────────────────────────
        self.steer_motor = rev.SparkMax(steer_id, rev.SparkMax.MotorType.kBrushless)
        steer_cfg = self._build_steer_config(steer_inverted)
        self.steer_motor.configure(
            steer_cfg,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )
        self.steer_encoder = self.steer_motor.getEncoder()
        self.steer_pid     = self.steer_motor.getClosedLoopController()

        # ── CANcoder ──────────────────────────────────────────
        self.cancoder = ph6_hw.CANcoder(cancoder_id)
        self._offset_degrees = cancoder_offset_degrees
        self._configure_cancoder()

        # Seed SparkMAX relative encoder from CANcoder absolute position
        self._sync_steer_encoder()

    # ── Motor config builders ─────────────────────────────────

    def _build_drive_config(self, inverted: bool) -> rev.SparkMaxConfig:
        cfg = rev.SparkMaxConfig()
        cfg.inverted(inverted)
        cfg.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        cfg.smartCurrentLimit(50)
        cfg.encoder.positionConversionFactor(SwerveConstants.DRIVE_ENC_POSITION_FACTOR)
        cfg.encoder.velocityConversionFactor(SwerveConstants.DRIVE_ENC_VELOCITY_FACTOR)
        cfg.closedLoop.pid(
            SwerveConstants.DRIVE_KP,
            SwerveConstants.DRIVE_KI,
            SwerveConstants.DRIVE_KD,
        )
        cfg.closedLoop.velocityFF(SwerveConstants.DRIVE_KFF)
        cfg.closedLoop.outputRange(-1, 1)
        return cfg

    def _build_steer_config(self, inverted: bool) -> rev.SparkMaxConfig:
        cfg = rev.SparkMaxConfig()
        cfg.inverted(inverted)
        cfg.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        cfg.smartCurrentLimit(20)
        cfg.encoder.positionConversionFactor(SwerveConstants.STEER_ENC_POSITION_FACTOR)
        cfg.encoder.velocityConversionFactor(SwerveConstants.STEER_ENC_VELOCITY_FACTOR)
        cfg.closedLoop.pid(
            SwerveConstants.STEER_KP,
            SwerveConstants.STEER_KI,
            SwerveConstants.STEER_KD,
        )
        # ✅ Wrapping disabled — we handle shortest path manually above
        cfg.closedLoop.positionWrappingEnabled(False)
        cfg.closedLoop.outputRange(-1, 1)
        return cfg

    def _configure_cancoder(self):
        cc_cfg = configs.CANcoderConfiguration()
        cc_cfg.magnet_sensor.magnet_offset = self._offset_degrees / 360.0
        cc_cfg.magnet_sensor.sensor_direction = signals.SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
        cc_cfg.magnet_sensor.absolute_sensor_discontinuity_point = 0.5
        self.cancoder.configurator.apply(cc_cfg)

    def _sync_steer_encoder(self):
        """Seed SparkMAX relative encoder from CANcoder absolute position on boot."""
        abs_rotations = self.cancoder.get_absolute_position().wait_for_update(0.1).value
        self.steer_encoder.setPosition(abs_rotations * 2 * math.pi)

    # ── State control ─────────────────────────────────────────

   
    def set_desired_state(self, desired_state: SwerveModuleState) -> None:
        current_angle = Rotation2d(self.steer_encoder.getPosition())
        desired_state.optimize(current_angle)

        # If angle error is large AND speed is crossing zero, hold wheel position
        angle_error = abs(desired_state.angle.radians() - current_angle.radians())
        if angle_error > math.pi / 2:
            self.drive_motor.set(0)
            return

        cos_scale = math.cos(desired_state.angle.radians() - current_angle.radians())

        self.drive_pid.setReference(
            desired_state.speed * cos_scale,
            rev.SparkMax.ControlType.kVelocity,
        )
        self.steer_pid.setReference(
            desired_state.angle.radians(),
            rev.SparkMax.ControlType.kPosition,
        )

    def stop(self) -> None:
        self.drive_motor.set(0)
        self.steer_motor.set(0)

    # ── State getters ─────────────────────────────────────────

    def get_state(self) -> SwerveModuleState:
        return SwerveModuleState(
            self.drive_encoder.getVelocity(),
            Rotation2d(self.steer_encoder.getPosition()),
        )

    def get_position(self) -> SwerveModulePosition:
        return SwerveModulePosition(
            self.drive_encoder.getPosition(),
            Rotation2d(self.steer_encoder.getPosition()),
        )

    def reset_drive_encoder(self) -> None:
        self.drive_encoder.setPosition(0)

    def get_cancoder_degrees(self) -> float:
        return self.cancoder.get_absolute_position().value * 360.0
"""
constants.py — FRC Team 5421
Swerve drive constants for:
  - REV NEO (drive) + REV NEO 550 (steer) via SparkMAX
  - CANcoder (Phoenix 6) absolute steer encoder
  - NavX2-MXP IMU
  - SDS MK4i L2 modules (update gear ratios if different)

⚠️  Update CAN IDs and CANcoder offsets before deploying!
"""

import math
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics


class CANIDs:
    # ── Drive Motors (SparkMAX) ───────────────────────────────
    FRONT_LEFT_DRIVE  = 8
    FRONT_RIGHT_DRIVE = 2
    BACK_LEFT_DRIVE   = 6
    BACK_RIGHT_DRIVE  = 4

    # ── Steer Motors (SparkMAX) ───────────────────────────────
    FRONT_LEFT_STEER  = 7
    FRONT_RIGHT_STEER = 1
    BACK_LEFT_STEER   = 5
    BACK_RIGHT_STEER  = 3

    # ── CANcoders (Phoenix 6) ─────────────────────────────────
    FRONT_LEFT_CANCODER  = 18
    FRONT_RIGHT_CANCODER = 14
    BACK_LEFT_CANCODER   = 17
    BACK_RIGHT_CANCODER  = 15


class SwerveConstants:
    # ── Robot geometry (measure your actual robot!) ───────────
    WHEELBASE_METERS  = 0.762   # front-to-back  (23.5 in)
    TRACKWIDTH_METERS = 0.762   # left-to-right  (23.5 in)

    # ── Wheel ─────────────────────────────────────────────────
    WHEEL_DIAMETER_METERS = 0.10033   # 3.95 in SDS billet
    WHEEL_CIRCUMFERENCE   = math.pi * WHEEL_DIAMETER_METERS

    # ── Gear ratios (SDS MK4i L2) ────────────────────────────
    DRIVE_GEAR_RATIO  = 6.75       # motor rotations per wheel rotation
    STEER_GEAR_RATIO  = 150 / 7    # 21.43:1

    # ── Encoder conversion factors ────────────────────────────
    # SparkMAX built-in encoder is on the motor shaft (before gearbox)
    DRIVE_ENC_POSITION_FACTOR = WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO   # motor rot → meters
    DRIVE_ENC_VELOCITY_FACTOR = DRIVE_ENC_POSITION_FACTOR / 60.0         # RPM → m/s
    STEER_ENC_POSITION_FACTOR = (2 * math.pi) / STEER_GEAR_RATIO         # motor rot → radians
    STEER_ENC_VELOCITY_FACTOR = STEER_ENC_POSITION_FACTOR / 60.0         # RPM → rad/s

    # ── Max speeds ────────────────────────────────────────────
    MAX_DRIVE_SPEED_MPS   = 4.6    # ~14.8 ft/s (NEO free speed limited)
    MAX_ANGULAR_SPEED_RPS = 2 * math.pi   # one full rotation per second

    # ── CANcoder absolute offsets (DEGREES) ──────────────────
    # How to calibrate:
    #   1. Point all wheels straight forward
    #   2. Read each CANcoder's absolute position in Phoenix Tuner X
    #   3. Paste the values here (negate if wheel spins backwards)
    FRONT_LEFT_OFFSET  = -.1677*360 #-.1677 rotations
    FRONT_RIGHT_OFFSET =  0.56*360 # .06 rotations
    BACK_LEFT_OFFSET   = .029*360 # .029 rotations
    BACK_RIGHT_OFFSET  =  -.499*360 #-.499 rotations

    # ── Module positions relative to robot center ─────────────
    FRONT_LEFT_POSITION  = Translation2d( WHEELBASE_METERS / 2,  TRACKWIDTH_METERS / 2)
    FRONT_RIGHT_POSITION = Translation2d( WHEELBASE_METERS / 2, -TRACKWIDTH_METERS / 2)
    BACK_LEFT_POSITION   = Translation2d(-WHEELBASE_METERS / 2,  TRACKWIDTH_METERS / 2)
    BACK_RIGHT_POSITION  = Translation2d(-WHEELBASE_METERS / 2, -TRACKWIDTH_METERS / 2)

    KINEMATICS = SwerveDrive4Kinematics(
        FRONT_LEFT_POSITION,
        FRONT_RIGHT_POSITION,
        BACK_LEFT_POSITION,
        BACK_RIGHT_POSITION,
    )

    # ── Drive PID (SparkMAX onboard) ──────────────────────────
    DRIVE_KP  = 0.4
    DRIVE_KI  = 0.0
    DRIVE_KD  = 0.0
    DRIVE_KFF = 1 / (MAX_DRIVE_SPEED_MPS / DRIVE_ENC_VELOCITY_FACTOR)  # feedforward

    # ── Steer PID (SparkMAX onboard) ─────────────────────────
    STEER_KP  = 1.5
    STEER_KI  = 0.0
    STEER_KD  = 0.0


class OI:
    DRIVER_PORT     = 0
    DRIVER_DEADBAND = 0.05

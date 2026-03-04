import commands2
from rev import CANSparkMax, CANSparkLowLevel


class Climber(commands2.SubsystemBase):
    """
    Climber subsystem — two mirrored NEO motors on SparkMax controllers
    that pull simultaneously to raise/lower the robot for climbing.
    """

    # ── Configuration ──────────────────────────────────────────────
    LEFT_MOTOR_CAN_ID = 9         # TODO: set to your actual left CAN ID
    RIGHT_MOTOR_CAN_ID = 10       # TODO: set to your actual right CAN ID
    CLIMB_SPEED = 0.8             # Motor output for extending  (0.0–1.0)
    RETRACT_SPEED = -0.8          # Motor output for retracting
    LEFT_INVERTED = False          # TODO: tune — one side will likely need to be flipped
    RIGHT_INVERTED = True          # Mirrored motors typically spin opposite directions
    CURRENT_LIMIT = 40             # Amps per motor

    def __init__(self) -> None:
        super().__init__()

        # Left motor setup
        self.left_motor = CANSparkMax(
            self.LEFT_MOTOR_CAN_ID,
            CANSparkLowLevel.MotorType.kBrushless,
        )
        self.left_motor.restoreFactoryDefaults()
        self.left_motor.setInverted(self.LEFT_INVERTED)
        self.left_motor.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.left_motor.setSmartCurrentLimit(self.CURRENT_LIMIT)

        # Right motor setup
        self.right_motor = CANSparkMax(
            self.RIGHT_MOTOR_CAN_ID,
            CANSparkLowLevel.MotorType.kBrushless,
        )
        self.right_motor.restoreFactoryDefaults()
        self.right_motor.setInverted(self.RIGHT_INVERTED)
        self.right_motor.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.right_motor.setSmartCurrentLimit(self.CURRENT_LIMIT)

    # ── Public control methods ─────────────────────────────────────
    def extend(self) -> None:
        """Run both motors to extend / raise the climb arms simultaneously."""
        self.left_motor.set(self.CLIMB_SPEED)
        self.right_motor.set(self.CLIMB_SPEED)

    def retract(self) -> None:
        """Run both motors to retract / lower the climb arms simultaneously."""
        self.left_motor.set(self.RETRACT_SPEED)
        self.right_motor.set(self.RETRACT_SPEED)

    def stop(self) -> None:
        """Stop both climb motors."""
        self.left_motor.set(0.0)
        self.right_motor.set(0.0)

    # ── Periodic (optional telemetry) ──────────────────────────────
    def periodic(self) -> None:
        """Called every scheduler loop — add SmartDashboard logging here if desired."""
        pass
import commands2
from rev import CANSparkMax, CANSparkLowLevel
from wpilib import XboxController


class Intake(commands2.SubsystemBase):

    LEFT_MOTOR_CAN_ID = 11        # TODO: set to your actual CAN ID
    RIGHT_MOTOR_CAN_ID = 12       # TODO: set to your actual CAN ID
    INTAKE_SPEED = 0.7
    LEFT_INVERTED = False
    RIGHT_INVERTED = True          # Mirrored motors
    CURRENT_LIMIT = 30

    def __init__(self, controller: XboxController) -> None:
        super().__init__()
        self.controller = controller

        # Left motor
        self.left_motor = CANSparkMax(
            self.LEFT_MOTOR_CAN_ID,
            CANSparkLowLevel.MotorType.kBrushless,
        )
        self.left_motor.restoreFactoryDefaults()
        self.left_motor.setInverted(self.LEFT_INVERTED)
        self.left_motor.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.left_motor.setSmartCurrentLimit(self.CURRENT_LIMIT)

        # Right motor
        self.right_motor = CANSparkMax(
            self.RIGHT_MOTOR_CAN_ID,
            CANSparkLowLevel.MotorType.kBrushless,
        )
        self.right_motor.restoreFactoryDefaults()
        self.right_motor.setInverted(self.RIGHT_INVERTED)
        self.right_motor.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.right_motor.setSmartCurrentLimit(self.CURRENT_LIMIT)

    def periodic(self) -> None:
        if self.controller.getRightTriggerAxis() > 0.5:    # Right trigger pulled
            self.left_motor.set(self.INTAKE_SPEED)
            self.right_motor.set(self.INTAKE_SPEED)
        else:                                                # Released
            self.left_motor.set(0.0)
            self.right_motor.set(0.0)

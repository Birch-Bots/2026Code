import commands2
from rev import CANSparkMax, CANSparkLowLevel


class Intake(commands2.SubsystemBase):

    MOTOR_CAN_ID = 11             # TODO: set to your actual CAN ID
    INTAKE_SPEED = 0.7            # TODO: tune to your needs
    MOTOR_INVERTED = False
    CURRENT_LIMIT = 30

    def __init__(self) -> None:
        super().__init__()

        self.motor = CANSparkMax(
            self.MOTOR_CAN_ID,
            CANSparkLowLevel.MotorType.kBrushless,
        )
        self.motor.restoreFactoryDefaults()
        self.motor.setInverted(self.MOTOR_INVERTED)
        self.motor.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.motor.setSmartCurrentLimit(self.CURRENT_LIMIT)

    def intake(self) -> None:
        self.motor.set(self.INTAKE_SPEED)

    def stop(self) -> None:
        self.motor.set(0.0)

    def periodic(self) -> None:
        pass

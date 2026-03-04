import commands2
from rev import CANSparkMax, CANSparkLowLevel
from wpilib import XboxController


class Climber(commands2.SubsystemBase):

    LEFT_MOTOR_CAN_ID = 9         # TODO: set to your actual CAN ID
    RIGHT_MOTOR_CAN_ID = 10       # TODO: set to your actual CAN ID
    CLIMB_SPEED = 0.8
    RETRACT_SPEED = -0.8
    LEFT_INVERTED = False
    RIGHT_INVERTED = True
    CURRENT_LIMIT = 40

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
        pov = self.controller.getPOV()

        if pov == 0:        # D-pad up
            self.left_motor.set(self.CLIMB_SPEED)
            self.right_motor.set(self.CLIMB_SPEED)
        elif pov == 180:    # D-pad down
            self.left_motor.set(self.RETRACT_SPEED)
            self.right_motor.set(self.RETRACT_SPEED)
        else:               # Nothing pressed
            self.left_motor.set(0.0)
            self.right_motor.set(0.0)

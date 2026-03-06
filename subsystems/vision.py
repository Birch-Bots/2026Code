import commands2
from ntcore import NetworkTableInstance
from wpilib import SmartDashboard


class Vision(commands2.SubsystemBase):

    def __init__(self) -> None:
        super().__init__()

        # Connect to Limelight's NetworkTable
        self.table = NetworkTableInstance.getDefault().getTable("limelight")

    def has_target(self) -> bool:
        """Returns True if Limelight sees an AprilTag."""
        return self.table.getNumber("tv", 0) == 1

    def get_tag_id(self) -> int:
        """Returns the ID of the primary AprilTag in view."""
        return int(self.table.getNumber("tid", -1))

    def get_tx(self) -> float:
        """Horizontal offset from crosshair to target in degrees."""
        return self.table.getNumber("tx", 0.0)

    def get_ty(self) -> float:
        """Vertical offset from crosshair to target in degrees."""
        return self.table.getNumber("ty", 0.0)

    def get_ta(self) -> float:
        """Target area (0% to 100% of image)."""
        return self.table.getNumber("ta", 0.0)

    def get_botpose(self) -> list:
        """
        Returns robot pose on the field from AprilTag detection.
        [x, y, z, roll, pitch, yaw] in meters and degrees.
        """
        return self.table.getNumberArray("botpose", [0, 0, 0, 0, 0, 0])

    def get_botpose_blue(self) -> list:
        """Robot pose relative to blue alliance origin."""
        return self.table.getNumberArray("botpose_wpiblue", [0, 0, 0, 0, 0, 0])

    def get_botpose_red(self) -> list:
        """Robot pose relative to red alliance origin."""
        return self.table.getNumberArray("botpose_wpired", [0, 0, 0, 0, 0, 0])

    def set_pipeline(self, pipeline: int) -> None:
        """Switch Limelight pipeline (0-9)."""
        self.table.putNumber("pipeline", pipeline)

    def set_led_mode(self, mode: int) -> None:
        """
        Set LED mode.
        0 = use pipeline setting
        1 = force off
        2 = force blink
        3 = force on
        """
        self.table.putNumber("ledMode", mode)

    def periodic(self) -> None:
        SmartDashboard.putBoolean("Vision/HasTarget", self.has_target())
        SmartDashboard.putNumber("Vision/TagID", self.get_tag_id())
        SmartDashboard.putNumber("Vision/TX", self.get_tx())
        SmartDashboard.putNumber("Vision/TY", self.get_ty())

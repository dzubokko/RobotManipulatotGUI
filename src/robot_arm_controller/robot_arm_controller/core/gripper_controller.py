from typing import Callable

from robot_arm_controller.core.robot_model import RobotModel


class GripperController:

    def __init__(
        self,
        model: RobotModel,
        publish_gripper_target: Callable[[float, float], bool],
        logger=None,
    ) -> None:
        self.model = model
        self.publish_gripper_target = publish_gripper_target
        self.logger = logger
        self.last_target = self.model.gripper_closed_position

    def _info(self, message: str) -> None:
        if self.logger is not None:
            self.logger.info(message)

    def set_opening(self, opening: float, duration_sec: float = 0.7) -> bool:
        opening = self.model.clamp_gripper_opening(opening)
        self.last_target = opening
        self._info(
            f"Set gripper opening: {opening:.4f} m, duration={duration_sec:.2f}s"
        )
        return self.publish_gripper_target(opening, duration_sec)

    def open(self, duration_sec: float = 0.7) -> bool:
        return self.set_opening(self.model.gripper_open_position, duration_sec)

    def close(self, duration_sec: float = 0.7) -> bool:
        return self.set_opening(self.model.gripper_closed_position, duration_sec)
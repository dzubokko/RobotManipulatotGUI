import os
import signal
import subprocess
from pathlib import Path
from typing import Optional


class SimulatorManager:
    """
    Utility for starting and stopping Gazebo from the GUI.

    The current ManualControlUI still has its own launch logic. This class is
    prepared as the engineering replacement so that simulation management can
    later be moved out of the UI layer.
    """

    def __init__(self, workspace_root: Optional[Path] = None, logger=None) -> None:
        self.workspace_root = workspace_root or self.find_workspace_root()
        self.logger = logger
        self.process: Optional[subprocess.Popen] = None

    def _info(self, message: str) -> None:
        if self.logger is not None:
            self.logger.info(message)
        else:
            print(message)

    def _warn(self, message: str) -> None:
        if self.logger is not None:
            self.logger.warn(message)
        else:
            print(message)

    @staticmethod
    def find_workspace_root() -> Path:
        current = Path.cwd().resolve()
        candidates = [current] + list(current.parents) + [Path.home() / "RobotManipulator"]

        for candidate in candidates:
            launch_file = candidate / "src" / "robot_gazebo" / "launch" / "gazebo.launch.py"
            if launch_file.exists():
                return candidate

        return Path.home() / "RobotManipulator"

    def start(self) -> bool:
        if self.process is not None and self.process.poll() is None:
            self._warn("Gazebo is already running.")
            return True

        setup_file = self.workspace_root / "install" / "setup.bash"
        if not setup_file.exists():
            self._warn(
                f"{setup_file} not found. Build the workspace before starting Gazebo."
            )
            return False

        command = (
            f'cd "{self.workspace_root}" && '
            "source /opt/ros/humble/setup.bash && "
            "source install/setup.bash && "
            "ros2 launch robot_gazebo gazebo.launch.py"
        )

        self.process = subprocess.Popen(
            ["bash", "-lc", command],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
            text=True,
        )
        self._info("Gazebo launch command started.")
        return True

    def stop(self) -> None:
        if self.process is None or self.process.poll() is not None:
            self.cleanup_processes()
            self.process = None
            self._info("Gazebo is not running.")
            return

        try:
            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            self.process.wait(timeout=5)
        except Exception:
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
            except Exception as exc:
                self._warn(f"Failed to kill Gazebo process group: {exc}")

        self.cleanup_processes()
        self.process = None
        self._info("Gazebo stopped.")

    @staticmethod
    def cleanup_processes() -> None:
        cleanup_command = (
            "pkill -TERM -f 'ros2 launch robot_gazebo gazebo.launch.py' || true; "
            "pkill -TERM -f 'gzserver' || true; "
            "pkill -TERM -f 'gzclient' || true; "
            "pkill -TERM -f 'gazebo' || true"
        )
        subprocess.run(
            ["bash", "-lc", cleanup_command],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=3,
            check=False,
        )
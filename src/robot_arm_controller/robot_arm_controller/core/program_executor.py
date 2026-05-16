import json
import math
import re
import threading
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple


class ProgramParseError(Exception):
    pass


class ProgramStopped(Exception):
    pass


@dataclass
class SourceLine:
    number: int
    indent: int
    text: str


@dataclass
class ProgramCommand:
    line_no: int
    name: str
    args: List[float]
    raw: str


class RobotProgramParser:
    NUMBER_RE = re.compile(
        r"^[+-]?(?:(?:\d+(?:\.\d*)?)|(?:\.\d+))(?:[eE][+-]?\d+)?$"
    )
    CALL_RE = re.compile(r"^([A-Za-z_]\w*)\s*\((.*)\)\s*$")
    LOOP_RE = re.compile(
        r"^for\s+([A-Za-z_]\w*)\s+in\s+range\s*\((.*?)\)\s*:\s*$"
    )

    VALID_COMMANDS = {
        "move_lin",
        "rotate_rx",
        "rotate_ry",
        "rotate_rz",
        "joint_set",
        "wait",
        "reset_home",
        "grip_open",
        "grip_close",
        "grip_set",
        "save_ref",
        "align_to_ref",
        "stop_motion",
    }

    def parse(self, source: str) -> List[ProgramCommand]:
        lines = self._prepare_lines(source)

        if not lines:
            return []

        commands, index = self._parse_block(lines, 0, lines[0].indent, {})

        if index != len(lines):
            line = lines[index]
            raise ProgramParseError(
                f"Строка {line.number}: невозможно разобрать строку: {line.text}"
            )

        self.validate_commands(commands)
        return commands

    def validate_source(
        self,
        source: str,
    ) -> Tuple[bool, List[str], List[ProgramCommand]]:
        try:
            commands = self.parse(source)
            messages = [
                "OK: программа валидна.",
                f"Команд: {len(commands)}",
            ]
            return True, messages, commands
        except ProgramParseError as exc:
            return False, [str(exc)], []

    def _prepare_lines(self, source: str) -> List[SourceLine]:
        result: List[SourceLine] = []

        for line_no, raw in enumerate(source.splitlines(), start=1):
            raw = raw.replace("\t", "    ")
            code = raw.split("#", 1)[0].rstrip()

            if not code.strip():
                continue

            indent = len(code) - len(code.lstrip(" "))
            text = code.strip()

            result.append(
                SourceLine(
                    number=line_no,
                    indent=indent,
                    text=text,
                )
            )

        return result

    def _parse_block(
        self,
        lines: List[SourceLine],
        index: int,
        indent: int,
        variables: Dict[str, float],
    ) -> Tuple[List[ProgramCommand], int]:
        commands: List[ProgramCommand] = []

        while index < len(lines):
            line = lines[index]

            if line.indent < indent:
                break

            if line.indent > indent:
                raise ProgramParseError(
                    f"Строка {line.number}: лишний отступ перед командой: {line.text}"
                )

            loop_match = self.LOOP_RE.match(line.text)

            if loop_match:
                var_name, range_args_text = loop_match.groups()
                range_values = self._parse_range_args(
                    range_args_text,
                    variables,
                    line.number,
                )

                if index + 1 >= len(lines):
                    raise ProgramParseError(
                        f"Строка {line.number}: цикл for не содержит тела."
                    )

                body_indent = lines[index + 1].indent

                if body_indent <= line.indent:
                    raise ProgramParseError(
                        f"Строка {line.number}: тело цикла должно иметь отступ."
                    )

                body_start = index + 1
                body_end = body_start

                while body_end < len(lines) and lines[body_end].indent >= body_indent:
                    body_end += 1

                body_lines = lines[body_start:body_end]

                for value in range_values:
                    next_variables = dict(variables)
                    next_variables[var_name] = float(value)

                    body_commands, parsed_to = self._parse_block(
                        body_lines,
                        0,
                        body_indent,
                        next_variables,
                    )

                    if parsed_to != len(body_lines):
                        bad = body_lines[parsed_to]
                        raise ProgramParseError(
                            f"Строка {bad.number}: ошибка внутри цикла."
                        )

                    commands.extend(body_commands)

                index = body_end
                continue

            command = self._parse_call(line, variables)
            commands.append(command)
            index += 1

        return commands, index

    def _parse_range_args(
        self,
        args_text: str,
        variables: Dict[str, float],
        line_no: int,
    ) -> List[int]:
        parts = [part.strip() for part in args_text.split(",") if part.strip()]

        if not 1 <= len(parts) <= 3:
            raise ProgramParseError(
                f"Строка {line_no}: range() принимает 1, 2 или 3 аргумента."
            )

        values = [
            int(self._parse_numeric_token(part, variables, line_no))
            for part in parts
        ]

        if len(values) == 1:
            start, stop, step = 0, values[0], 1
        elif len(values) == 2:
            start, stop = values
            step = 1
        else:
            start, stop, step = values

        if step == 0:
            raise ProgramParseError(
                f"Строка {line_no}: шаг range() не может быть 0."
            )

        generated = list(range(start, stop, step))

        if len(generated) > 1000:
            raise ProgramParseError(
                f"Строка {line_no}: цикл слишком большой, максимум 1000 итераций."
            )

        return generated

    def _parse_call(
        self,
        line: SourceLine,
        variables: Dict[str, float],
    ) -> ProgramCommand:
        match = self.CALL_RE.match(line.text)

        if not match:
            raise ProgramParseError(
                f"Строка {line.number}: неверный синтаксис команды: {line.text}"
            )

        name, args_text = match.groups()

        if name not in self.VALID_COMMANDS:
            raise ProgramParseError(
                f"Строка {line.number}: неизвестная команда: {name}"
            )

        args = self._parse_arguments(args_text, variables, line.number)

        return ProgramCommand(
            line_no=line.number,
            name=name,
            args=args,
            raw=line.text,
        )

    def _parse_arguments(
        self,
        args_text: str,
        variables: Dict[str, float],
        line_no: int,
    ) -> List[float]:
        if not args_text.strip():
            return []

        parts = [part.strip() for part in args_text.split(",")]
        args: List[float] = []

        for part in parts:
            if not part:
                raise ProgramParseError(
                    f"Строка {line_no}: пустой аргумент в списке параметров."
                )

            args.append(self._parse_numeric_token(part, variables, line_no))

        return args

    def _parse_numeric_token(
        self,
        token: str,
        variables: Dict[str, float],
        line_no: int,
    ) -> float:
        token = token.strip()

        if token.startswith("$"):
            var_name = token[1:]

            if var_name not in variables:
                raise ProgramParseError(
                    f"Строка {line_no}: переменная {token} не определена."
                )

            return float(variables[var_name])

        if not self.NUMBER_RE.match(token):
            raise ProgramParseError(
                f"Строка {line_no}: аргумент должен быть числом или переменной, получено: {token}"
            )

        return float(token)

    def validate_commands(self, commands: List[ProgramCommand]) -> None:
        for command in commands:
            self._validate_command(command)

    def _validate_command(self, command: ProgramCommand) -> None:
        name = command.name
        args = command.args
        line = command.line_no

        if name == "move_lin":
            self._require_arg_count(command, 4)
            dx, dy, dz, duration = args
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)

            if duration <= 0:
                raise ProgramParseError(
                    f"Строка {line}: duration должен быть больше 0."
                )

            if distance > 1.0:
                raise ProgramParseError(
                    f"Строка {line}: move_lin слишком большой ({distance:.3f} м)."
                )

        elif name in {"rotate_rx", "rotate_ry", "rotate_rz"}:
            self._require_arg_count(command, 2)
            angle_deg, duration = args

            if abs(angle_deg) > 360:
                raise ProgramParseError(
                    f"Строка {line}: угол слишком большой ({angle_deg}°)."
                )

            if duration <= 0:
                raise ProgramParseError(
                    f"Строка {line}: duration должен быть больше 0."
                )

        elif name == "joint_set":
            self._require_arg_count(command, 3)
            joint_index, angle_deg, duration = args

            if int(joint_index) != joint_index:
                raise ProgramParseError(
                    f"Строка {line}: номер сустава должен быть целым числом."
                )

            if not 1 <= int(joint_index) <= 6:
                raise ProgramParseError(
                    f"Строка {line}: номер сустава должен быть от 1 до 6."
                )

            if abs(angle_deg) > 360:
                raise ProgramParseError(
                    f"Строка {line}: угол сустава слишком большой ({angle_deg}°)."
                )

            if duration <= 0:
                raise ProgramParseError(
                    f"Строка {line}: duration должен быть больше 0."
                )

        elif name == "wait":
            self._require_arg_count(command, 1)
            duration = args[0]

            if duration < 0:
                raise ProgramParseError(
                    f"Строка {line}: wait не может быть отрицательным."
                )

            if duration > 3600:
                raise ProgramParseError(
                    f"Строка {line}: wait слишком большой."
                )

        elif name == "reset_home":
            self._require_arg_count_range(command, 0, 1)

            if args and args[0] <= 0:
                raise ProgramParseError(
                    f"Строка {line}: duration должен быть больше 0."
                )

        elif name in {"grip_open", "grip_close"}:
            self._require_arg_count_range(command, 0, 1)

            if args and args[0] <= 0:
                raise ProgramParseError(
                    f"Строка {line}: duration должен быть больше 0."
                )

        elif name == "grip_set":
            self._require_arg_count_range(command, 1, 2)
            opening = args[0]
            duration = args[1] if len(args) > 1 else 0.7

            if not 0.0 <= opening <= 0.025:
                raise ProgramParseError(
                    f"Строка {line}: раскрытие захвата должно быть от 0 до 0.025 м."
                )

            if duration <= 0:
                raise ProgramParseError(
                    f"Строка {line}: duration должен быть больше 0."
                )

        elif name == "save_ref":
            self._require_arg_count(command, 0)

        elif name == "align_to_ref":
            self._require_arg_count_range(command, 0, 1)

            if args and args[0] <= 0:
                raise ProgramParseError(
                    f"Строка {line}: duration должен быть больше 0."
                )

        elif name == "stop_motion":
            self._require_arg_count(command, 0)

        else:
            raise ProgramParseError(
                f"Строка {line}: неизвестная команда: {name}"
            )

    @staticmethod
    def _require_arg_count(command: ProgramCommand, expected: int) -> None:
        actual = len(command.args)

        if actual != expected:
            raise ProgramParseError(
                f"Строка {command.line_no}: команда {command.name} требует "
                f"{expected} арг., получено {actual}."
            )

    @staticmethod
    def _require_arg_count_range(
        command: ProgramCommand,
        minimum: int,
        maximum: int,
    ) -> None:
        actual = len(command.args)

        if not minimum <= actual <= maximum:
            raise ProgramParseError(
                f"Строка {command.line_no}: команда {command.name} требует "
                f"от {minimum} до {maximum} арг., получено {actual}."
            )

    @staticmethod
    def commands_to_json(commands: List[ProgramCommand]) -> List[Dict]:
        return [asdict(command) for command in commands]

    @staticmethod
    def legacy_json_commands_to_source(commands: List[Dict]) -> str:
        lines: List[str] = []

        for command in commands:
            ctype = command.get("type")

            if ctype == "move":
                dx, dy, dz = command.get("target", [0.0, 0.0, 0.0])
                duration = command.get("speed", 1.0)
                lines.append(f"move_lin({dx}, {dy}, {dz}, {duration})")

            elif ctype == "rotate_rx":
                lines.append(
                    f"rotate_rx({command.get('angle', 0.0)}, {command.get('duration', 1.0)})"
                )

            elif ctype == "rotate_ry":
                lines.append(
                    f"rotate_ry({command.get('angle', 0.0)}, {command.get('duration', 1.0)})"
                )

            elif ctype == "rotate_rz":
                lines.append(
                    f"rotate_rz({command.get('angle', 0.0)}, {command.get('duration', 1.0)})"
                )

            elif ctype == "joint_set":
                lines.append(
                    f"joint_set({command.get('joint_idx', 1)}, "
                    f"{command.get('angle', 0.0)}, "
                    f"{command.get('duration', 1.0)})"
                )

            elif ctype == "reset_home":
                lines.append("reset_home()")

            elif ctype == "wait":
                lines.append(f"wait({command.get('duration', 1.0)})")

            elif ctype == "gripper_open":
                lines.append("grip_open()")

            elif ctype == "gripper_close":
                lines.append("grip_close()")

            elif ctype == "save_ref":
                lines.append("save_ref()")

            elif ctype == "align_to_ref":
                lines.append(f"align_to_ref({command.get('duration', 0.3)})")

        return "\n".join(lines)


class RobotProgramExecutor:
    def __init__(
        self,
        robot,
        log_callback: Optional[Callable[[str, str], None]] = None,
        progress_callback: Optional[Callable[[int, int, ProgramCommand], None]] = None,
    ) -> None:
        self.robot = robot
        self.log_callback = log_callback
        self.progress_callback = progress_callback
        self._stop_event = threading.Event()
        self.strict_motion_wait = False

    def request_stop(self) -> None:
        self._stop_event.set()

        try:
            if self.robot is not None and hasattr(self.robot, "stop_motion"):
                self.robot.stop_motion()
        except Exception:
            pass

    def reset_stop(self) -> None:
        self._stop_event.clear()

    def run_source(self, source: str) -> None:
        parser = RobotProgramParser()
        commands = parser.parse(source)
        self.run_commands(commands)

    def run_commands(self, commands: List[ProgramCommand]) -> None:
        self.reset_stop()

        total = len(commands)

        if total == 0:
            self._log("Программа не содержит команд.", "WARN")
            return

        self._log(f"Старт выполнения программы. Команд: {total}")

        for index, command in enumerate(commands, start=1):
            self._check_stop()
            self._check_estop()

            if self.progress_callback is not None:
                self.progress_callback(index, total, command)

            self._log(f"[{index}/{total}] Строка {command.line_no}: {command.raw}")
            self.execute_command(command)

        self._log("Программа выполнена успешно.")

    def execute_command(self, command: ProgramCommand) -> None:
        if self.robot is None:
            raise RuntimeError("RobotBridge не подключён.")

        name = command.name
        args = command.args

        if name == "move_lin":
            dx, dy, dz, duration = args
            success = self.robot.move_end_effector_world(
                -dx,
                -dy,
                dz,
                duration=duration,
            )

            if not success:
                self._log(
                    "move_lin: IK не сошлась полностью или движение выполнено приближённо.",
                    "WARN",
                )

            self._wait_robot_motion(duration)

        elif name == "rotate_rx":
            angle_deg, duration = args
            angle_rad = math.radians(angle_deg)

            success = self.robot.rotate_end_effector_rx_ry_ik(
                d_rx=angle_rad,
                d_ry=0.0,
                duration=duration,
            )

            if not success:
                self._log("rotate_rx: IK не сошлась полностью.", "WARN")

            self._wait_robot_motion(duration)

        elif name == "rotate_ry":
            angle_deg, duration = args
            angle_rad = math.radians(angle_deg)

            success = self.robot.rotate_end_effector_rx_ry_ik(
                d_rx=0.0,
                d_ry=angle_rad,
                duration=duration,
            )

            if not success:
                self._log("rotate_ry: IK не сошлась полностью.", "WARN")

            self._wait_robot_motion(duration)

        elif name == "rotate_rz":
            angle_deg, duration = args
            angle_rad = math.radians(angle_deg)

            success = self.robot.rotate_end_effector_world(
                drz=angle_rad,
                duration=duration,
            )

            if not success:
                self._log("rotate_rz: IK не сошлась полностью.", "WARN")

            self._wait_robot_motion(duration)

        elif name == "joint_set":
            joint_index, angle_deg, duration = args
            angle_rad = math.radians(angle_deg)

            success = self.robot.move_joint(
                int(joint_index),
                angle_rad,
                duration_sec=duration,
            )

            if not success:
                raise RuntimeError("joint_set: команда не отправлена.")

            self._wait_robot_motion(duration)

        elif name == "reset_home":
            duration = args[0] if args else 2.0
            success = self.robot.reset_position()

            if not success:
                raise RuntimeError("reset_home: команда не отправлена.")

            self._wait_robot_motion(duration)

        elif name == "wait":
            duration = args[0]
            self._log(f"Ожидание {duration:.2f} сек.")
            self._safe_sleep(duration)

        elif name == "grip_open":
            duration = args[0] if args else 0.7
            success = self.robot.open_gripper(duration)

            if not success:
                raise RuntimeError("grip_open: команда не отправлена.")

            self._safe_sleep(duration + 0.1)

        elif name == "grip_close":
            duration = args[0] if args else 0.7
            success = self.robot.close_gripper(duration)

            if not success:
                raise RuntimeError("grip_close: команда не отправлена.")

            self._safe_sleep(duration + 0.1)

        elif name == "grip_set":
            opening = args[0]
            duration = args[1] if len(args) > 1 else 0.7
            success = self.robot.set_gripper(opening, duration)

            if not success:
                raise RuntimeError("grip_set: команда не отправлена.")

            self._safe_sleep(duration + 0.1)

        elif name == "save_ref":
            self.robot.save_reference_orientation()
            self._safe_sleep(0.2)

        elif name == "align_to_ref":
            duration = args[0] if args else 0.3
            success = self.robot.align_orientation_to_reference(duration)

            if not success:
                self._log("align_to_ref: IK не сошлась полностью.", "WARN")

            self._wait_robot_motion(duration)

        elif name == "stop_motion":
            self.robot.stop_motion()
            self._safe_sleep(0.1)

        else:
            raise RuntimeError(f"Неизвестная команда: {name}")

    def _wait_robot_motion(self, duration: float) -> None:
        self._check_stop()
        self._check_estop()

        wait_timeout = max(0.5, duration + 2.0)
        reached = True

        if hasattr(self.robot, "wait_until_arm_reached"):
            reached = self.robot.wait_until_arm_reached(timeout_sec=wait_timeout)
        else:
            self._safe_sleep(duration + 0.1)
            reached = True

        self._check_stop()
        self._check_estop()

        if reached:
            return

        diagnostic = self._get_motion_error_diagnostic()
        message = f"Робот не достиг целевой позиции за {wait_timeout:.2f} сек."

        if diagnostic:
            message += f" {diagnostic}"

        if self.strict_motion_wait:
            raise RuntimeError(message)

        self._log(message, "WARN")
        self._log(
            "Выполнение продолжено, так как включён мягкий режим ожидания для Gazebo.",
            "WARN",
        )

    def _get_motion_error_diagnostic(self) -> str:
        try:
            if not hasattr(self.robot, "get_status"):
                return ""

            status = self.robot.get_status()
            target = status.get("arm_target")

            if target is None:
                return ""

            current = self.robot.get_current_position()

            if not current or len(current) != len(target):
                return ""

            errors_rad = [
                abs(float(current[i]) - float(target[i]))
                for i in range(len(target))
            ]

            max_error_rad = max(errors_rad)
            max_error_deg = math.degrees(max_error_rad)
            joint_index = errors_rad.index(max_error_rad) + 1

            return f"Макс. ошибка: J{joint_index} = {max_error_deg:.3f}°."

        except Exception:
            return ""

    def _safe_sleep(self, duration: float) -> None:
        start = time.monotonic()

        while time.monotonic() - start < duration:
            self._check_stop()
            self._check_estop()
            time.sleep(0.02)

    def _check_stop(self) -> None:
        if self._stop_event.is_set():
            raise ProgramStopped("Выполнение программы остановлено пользователем.")

    def _check_estop(self) -> None:
        try:
            if self.robot is None or not hasattr(self.robot, "get_status"):
                return

            status = self.robot.get_status()

            if bool(status.get("e_stop_active", False)):
                raise ProgramStopped("Выполнение остановлено: активен Emergency Stop.")

        except ProgramStopped:
            raise
        except Exception:
            return

    def _log(self, message: str, level: str = "INFO") -> None:
        if self.log_callback is not None:
            self.log_callback(message, level)


class RobotProgramStorage:
    def __init__(self, programs_dir: Path) -> None:
        self.programs_dir = programs_dir
        self.programs_dir.mkdir(parents=True, exist_ok=True)

    def list_programs(self) -> List[str]:
        return sorted(path.stem for path in self.programs_dir.glob("*.robot"))

    def save_program(
        self,
        name: str,
        source: str,
        commands: List[ProgramCommand],
    ) -> Path:
        self._validate_program_name(name)

        path = self.programs_dir / f"{name}.robot"

        data = {
            "name": name,
            "format": "robot_dsl_v2",
            "description": "",
            "created_or_updated_at": datetime.now().isoformat(),
            "source": source,
            "commands": RobotProgramParser.commands_to_json(commands),
        }

        path.write_text(
            json.dumps(data, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

        return path

    def load_program(self, name: str) -> str:
        self._validate_program_name(name)

        path = self.programs_dir / f"{name}.robot"

        if not path.exists():
            raise FileNotFoundError(f"Файл не найден: {path}")

        text = path.read_text(encoding="utf-8")

        try:
            data = json.loads(text)

            if isinstance(data, dict):
                source = data.get("source")

                if isinstance(source, str):
                    return source

                legacy_commands = data.get("commands")

                if isinstance(legacy_commands, list):
                    return RobotProgramParser.legacy_json_commands_to_source(
                        legacy_commands
                    )

        except json.JSONDecodeError:
            pass

        return text

    def delete_program(self, name: str) -> Path:
        self._validate_program_name(name)

        path = self.programs_dir / f"{name}.robot"

        if not path.exists():
            raise FileNotFoundError(f"Файл не найден: {path}")

        path.unlink()
        return path

    @staticmethod
    def _validate_program_name(name: str) -> None:
        if not name:
            raise ValueError("Имя программы не может быть пустым.")

        if any(symbol in name for symbol in r'\/:*?"<>|'):
            raise ValueError("Имя программы содержит недопустимые символы.")
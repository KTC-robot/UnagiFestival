from unagifestival.tools.ps_controller.enum import ButtonState
from unagifestival.tools.ps_controller.model import ButtonEvent
from unagifestival.tools.ps_controller.servo.constants import (
    SERVO_ALL_CLOSE_ANGLE,
    SERVO_ALL_OPEN_ANGLE,
    SERVO_BUTTON_ACTIONS,
    SERVO_CHANNELS,
    SERVO_SEND_HOME_ON_START,
    SERVO_STARTUP_INTERVAL_SECONDS,
    SERVO_TOGGLE_ACTIONS,
)
from unagifestival.tools.ps_controller.servo.model import (
    ServoSetAllCommand,
    ServoSetCommand,
    ServoToggleKey,
)
from unagifestival.tools.ps_controller.servo.validate import validate_servo_config


class ServoMapper:
    """Controllerイベントを送信可能なサーボ指令へ変換する."""

    def __init__(self) -> None:
        validate_servo_config()
        self._toggle_state: dict[ServoToggleKey, bool] = {}

    @property
    def startup_interval_seconds(self) -> float:
        """起動時の個別サーボ指令間隔を返す."""
        return SERVO_STARTUP_INTERVAL_SECONDS

    def startup_commands(self) -> tuple[ServoSetCommand, ...]:
        """起動時に必要なhome指令を返す."""
        if not SERVO_SEND_HOME_ON_START:
            return ()
        return tuple(
            ServoSetCommand(channel, config.home_angle)
            for channel, config in enumerate(SERVO_CHANNELS)
            if config.enabled
        )

    def map_button(self, event: ButtonEvent) -> tuple[ServoSetCommand, ...]:
        """押下イベントをdirect/toggleサーボ指令へ変換する."""
        if event.state is not ButtonState.PRESSED:
            return ()

        commands: list[ServoSetCommand] = [
            ServoSetCommand(action.channel, action.angle)
            for action in SERVO_BUTTON_ACTIONS.get(event.code, ())
        ]
        for action_index, action in enumerate(
            SERVO_TOGGLE_ACTIONS.get(event.code, ())
        ):
            key = ServoToggleKey(event.code, action_index)
            use_angle_b = self._toggle_state.get(key, False)
            angle = action.angle_b if use_angle_b else action.angle_a
            self._toggle_state[key] = not use_angle_b
            commands.append(ServoSetCommand(action.channel, angle))
        return tuple(commands)

    @staticmethod
    def open_all() -> ServoSetAllCommand:
        """全サーボOPEN指令を返す."""
        return ServoSetAllCommand(SERVO_ALL_OPEN_ANGLE)

    @staticmethod
    def close_all() -> ServoSetAllCommand:
        """全サーボCLOSE指令を返す."""
        return ServoSetAllCommand(SERVO_ALL_CLOSE_ANGLE)

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
    """
    Properties:
        startup_interval_seconds: 起動時の個別Servo送信間隔。
    About:
        ControllerイベントをServo Commandへ変換し、toggle状態を管理する。
    """

    def __init__(self) -> None:
        """
        Args:
            なし。
        Returns:
            なし。
        About:
            Servo設定を検証し、toggle状態を初期化する。
        """
        validate_servo_config()
        self._toggle_state: dict[ServoToggleKey, bool] = {}

    @property
    def startup_interval_seconds(self) -> float:
        """
        Args:
            なし。
        Returns:
            起動時の個別Servo Command送信間隔を秒で表した値。
        About:
            設定済みの起動時送信間隔を公開する。
        """
        return SERVO_STARTUP_INTERVAL_SECONDS

    def startup_commands(self) -> tuple[ServoSetCommand, ...]:
        """
        Args:
            なし。
        Returns:
            有効な各Servoに対する起動時home Command。
        About:
            home送信が有効な場合だけ起動時Commandを生成する。
        """
        if not SERVO_SEND_HOME_ON_START:
            return ()
        return tuple(
            ServoSetCommand(channel, config.home_angle)
            for channel, config in enumerate(SERVO_CHANNELS)
            if config.enabled
        )

    def map_button(self, event: ButtonEvent) -> tuple[ServoSetCommand, ...]:
        """
        Args:
            event: 変換対象のControllerボタンイベント。
        Returns:
            direct操作とtoggle操作から生成したServo Command。
        About:
            押下イベントを設定済みのServo操作へ変換し、toggle状態を更新する。
        """
        if event.state is not ButtonState.PRESSED:
            return ()

        commands: list[ServoSetCommand] = [
            ServoSetCommand(action.channel, action.angle) for action in SERVO_BUTTON_ACTIONS.get(event.code, ())
        ]
        for action_index, action in enumerate(SERVO_TOGGLE_ACTIONS.get(event.code, ())):
            key = ServoToggleKey(event.code, action_index)
            use_angle_b = self._toggle_state.get(key, False)
            angle = action.angle_b if use_angle_b else action.angle_a
            self._toggle_state[key] = not use_angle_b
            commands.append(ServoSetCommand(action.channel, angle))
        return tuple(commands)

    @staticmethod
    def open_all() -> ServoSetAllCommand:
        """
        Args:
            なし。
        Returns:
            全ServoをOPEN角度へ設定するCommand。
        About:
            設定済みのOPEN角度を使った一括操作Commandを生成する。
        """
        return ServoSetAllCommand(SERVO_ALL_OPEN_ANGLE)

    @staticmethod
    def close_all() -> ServoSetAllCommand:
        """
        Args:
            なし。
        Returns:
            全ServoをCLOSE角度へ設定するCommand。
        About:
            設定済みのCLOSE角度を使った一括操作Commandを生成する。
        """
        return ServoSetAllCommand(SERVO_ALL_CLOSE_ANGLE)

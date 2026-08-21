"""Controller Hotfix behavior tests."""

# ruff: noqa: PT009, SLF001

import unittest

from unittest.mock import patch

from unagifestival.tools.ps_controller.enum import AxisCode, ButtonCode, ButtonState
from unagifestival.tools.ps_controller.handler.handler import Handler
from unagifestival.tools.ps_controller.im920.encoder import encode_command
from unagifestival.tools.ps_controller.im920.enum import Md20aState
from unagifestival.tools.ps_controller.im920.model import (
    AirFireStartCommand,
    AirFireStopCommand,
    EmergencyStopCommand,
    IM920Command,
    IM920Response,
    Md20aSetStateCommand,
    StepAssistManualFrontToggleCommand,
    StepAssistManualRearToggleCommand,
    StepAssistModeToggleCommand,
    StepAssistResetCommand,
    StopCommand,
)
from unagifestival.tools.ps_controller.model import (
    AxisInfo,
    AxisInputEvent,
    ButtonEvent,
    ControllerState,
)


class FakeIm920:
    def __init__(self) -> None:
        self.sent: list[IM920Command] = []

    def send(self, command: IM920Command) -> None:
        self.sent.append(command)

    def poll(self) -> IM920Response | None:
        return None

    def close(self) -> None:
        pass


def controller_state() -> ControllerState:
    info = {
        AxisCode.LEFT_STICK_X: AxisInfo(0, -32768, 32767),
        AxisCode.LEFT_STICK_Y: AxisInfo(0, -32768, 32767),
        AxisCode.RIGHT_STICK_X: AxisInfo(0, -32768, 32767),
        AxisCode.LEFT_TRIGGER_L2: AxisInfo(0, 0, 255),
        AxisCode.RIGHT_TRIGGER_R2: AxisInfo(0, 0, 255),
        AxisCode.DPAD_Y: AxisInfo(0, -1, 1),
    }
    return ControllerState(dict.fromkeys(info, 0), info)


class ControllerHotfixTest(unittest.TestCase):
    def setUp(self) -> None:
        self.im920 = FakeIm920()
        self.handler = Handler(self.im920)
        self.sleep_patch = patch("unagifestival.tools.ps_controller.handler.handler.time.sleep")
        self.sleep_patch.start()

    def tearDown(self) -> None:
        self.sleep_patch.stop()

    def press(self, button: ButtonCode) -> Md20aState:
        self.handler.handle_button(ButtonEvent(button, ButtonState.PRESSED))
        commands = self.im920.sent[-3:]
        self.assertEqual(len(commands), 3)
        self.assertTrue(all(isinstance(command, Md20aSetStateCommand) for command in commands))
        return commands[-1].state

    def test_r1_toggle_transitions(self) -> None:
        self.assertEqual(self.press(ButtonCode.R1_BTN), Md20aState.FORWARD)
        self.assertEqual(self.press(ButtonCode.R1_BTN), Md20aState.STOPPED)
        self.assertEqual(self.press(ButtonCode.L1_BTN), Md20aState.REVERSE)
        self.assertEqual(self.press(ButtonCode.R1_BTN), Md20aState.FORWARD)

    def test_l1_toggle_transitions(self) -> None:
        self.assertEqual(self.press(ButtonCode.L1_BTN), Md20aState.REVERSE)
        self.assertEqual(self.press(ButtonCode.L1_BTN), Md20aState.STOPPED)
        self.assertEqual(self.press(ButtonCode.R1_BTN), Md20aState.FORWARD)
        self.assertEqual(self.press(ButtonCode.L1_BTN), Md20aState.REVERSE)

    def test_r1_l1_release_does_nothing(self) -> None:
        for button in (ButtonCode.R1_BTN, ButtonCode.L1_BTN):
            self.handler.handle_button(ButtonEvent(button, ButtonState.RELEASED))
        self.assertEqual(self.im920.sent, [])

    def test_r2_sends_only_edges(self) -> None:
        state = controller_state()
        self.handler.handle_axis(AxisInputEvent(AxisCode.RIGHT_TRIGGER_R2, 100), state)
        self.assertEqual(len(self.im920.sent), 3)
        self.assertTrue(all(isinstance(command, AirFireStartCommand) for command in self.im920.sent))
        self.handler.handle_axis(AxisInputEvent(AxisCode.RIGHT_TRIGGER_R2, 200), state)
        self.assertEqual(len(self.im920.sent), 3)
        self.handler.handle_axis(AxisInputEvent(AxisCode.RIGHT_TRIGGER_R2, 0), state)
        self.assertEqual(len(self.im920.sent), 6)
        self.assertTrue(all(isinstance(command, AirFireStopCommand) for command in self.im920.sent[-3:]))

    def test_triggers_do_not_change_drive(self) -> None:
        state = controller_state()
        state.axis_values[AxisCode.LEFT_STICK_X] = 16000
        state.axis_values[AxisCode.LEFT_STICK_Y] = -12000
        state.axis_values[AxisCode.RIGHT_STICK_X] = 8000
        baseline = self.handler._make_drive_command(state)
        state.axis_values[AxisCode.LEFT_TRIGGER_L2] = 255
        state.axis_values[AxisCode.RIGHT_TRIGGER_R2] = 255
        self.assertEqual(self.handler._make_drive_command(state), baseline)

    def test_lateral_and_rotation_have_wider_deadzone_than_forward(self) -> None:
        # Given: 全スティック軸に中心から10%の入力がある。
        state = controller_state()
        state.axis_values[AxisCode.LEFT_STICK_X] = 3277
        state.axis_values[AxisCode.LEFT_STICK_Y] = 3277
        state.axis_values[AxisCode.RIGHT_STICK_X] = 3277

        # When: 走行指令へ変換する。
        command = self.handler._make_drive_command(state)

        # Then: 前後だけ反応し、左右移動と回転はデッドゾーン内になる。
        self.assertNotEqual(command.vx, 0)
        self.assertEqual(command.vy, 0)
        self.assertEqual(command.wz, 0)

    def test_state_command_wire_ids(self) -> None:
        self.assertEqual(encode_command(AirFireStartCommand()).payload, "430A")
        self.assertEqual(encode_command(AirFireStopCommand()).payload, "430B")
        self.assertEqual(
            encode_command(Md20aSetStateCommand(Md20aState.REVERSE)).payload,
            "430C02",
        )

    def test_ps_pressed_sends_mode_toggle_once(self) -> None:
        self.handler.handle_button(ButtonEvent(ButtonCode.PS_BTN, ButtonState.PRESSED))
        self.assertEqual(len(self.im920.sent), 1)
        self.assertIsInstance(self.im920.sent[0], StepAssistModeToggleCommand)
        self.assertNotIsInstance(self.im920.sent[0], EmergencyStopCommand)

    def test_ps_release_sends_nothing(self) -> None:
        self.handler.handle_button(ButtonEvent(ButtonCode.PS_BTN, ButtonState.RELEASED))
        self.assertEqual(self.im920.sent, [])

    def test_cross_stop_and_circle_reset_remain_mapped(self) -> None:
        self.handler.handle_button(ButtonEvent(ButtonCode.CROSS_BTN, ButtonState.PRESSED))
        self.handler.handle_button(ButtonEvent(ButtonCode.CIRCLE_BTN, ButtonState.PRESSED))
        self.assertIsInstance(self.im920.sent[0], StopCommand)
        self.assertIsInstance(self.im920.sent[1], StepAssistResetCommand)

    def test_dpad_y_sends_each_press_once_and_ignores_release(self) -> None:
        state = controller_state()
        self.handler.handle_axis(AxisInputEvent(AxisCode.DPAD_Y, -1), state)
        self.assertEqual(len(self.im920.sent), 1)
        self.assertIsInstance(self.im920.sent[-1], StepAssistManualFrontToggleCommand)

        self.handler.handle_axis(AxisInputEvent(AxisCode.DPAD_Y, -1), state)
        self.assertEqual(len(self.im920.sent), 1)
        self.handler.handle_axis(AxisInputEvent(AxisCode.DPAD_Y, 0), state)
        self.assertEqual(len(self.im920.sent), 1)

        self.handler.handle_axis(AxisInputEvent(AxisCode.DPAD_Y, 1), state)
        self.assertEqual(len(self.im920.sent), 2)
        self.assertIsInstance(self.im920.sent[-1], StepAssistManualRearToggleCommand)

    def test_step_assist_toggle_wire_ids(self) -> None:
        self.assertEqual(encode_command(StepAssistModeToggleCommand()).payload, "430D")
        self.assertEqual(
            encode_command(StepAssistManualFrontToggleCommand()).payload,
            "430E",
        )
        self.assertEqual(
            encode_command(StepAssistManualRearToggleCommand()).payload,
            "430F",
        )


if __name__ == "__main__":
    unittest.main()

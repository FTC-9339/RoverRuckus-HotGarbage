package com.roguemafia.rogueftc.control

import com.qualcomm.robotcore.hardware.Gamepad

data class JoypadConfig(
        val joystickDeadzone: Double = 0.2,
        val triggerDeadzone: Double = 0.0,
        val callback: () -> Unit = {
            // Empty as default
        }
)

class Joypad(private var config: JoypadConfig = JoypadConfig(), gamepad: Gamepad) {

    private var callback: Gamepad.GamepadCallback = Gamepad.GamepadCallback {
        val previousState = currentState
        currentState = JoypadState(
                left_stick = JoystickState(
                        x = it.left_stick_x.toDouble(),
                        y = it.left_stick_y.toDouble(),
                        clickyBoi = it.left_stick_button,
                        deadzone = config.joystickDeadzone
                ).cleanValues(),
                right_stick = JoystickState(
                        x = it.right_stick_x.toDouble(),
                        y = it.right_stick_y.toDouble(),
                        clickyBoi = it.right_stick_button,
                        deadzone = config.joystickDeadzone
                ).cleanValues(),
                dPadButtons = DPadState(
                        up = previousState.dPadButtons.up.updateButtonState(it.dpad_up),
                        down = previousState.dPadButtons.down.updateButtonState(it.dpad_down),
                        left = previousState.dPadButtons.left.updateButtonState(it.dpad_left),
                        right = previousState.dPadButtons.right.updateButtonState(it.dpad_right)
                ),
                faceButtons = FaceButtonState(
                        a = previousState.faceButtons.a.updateButtonState(it.a),
                        b = previousState.faceButtons.b.updateButtonState(it.b),
                        x = previousState.faceButtons.x.updateButtonState(it.x),
                        y = previousState.faceButtons.y.updateButtonState(it.y)
                ),
                triggerBumperState = TriggerBumperState(
                        left_trigger = TriggerState(
                                value = it.left_trigger.toDouble(),
                                deadzone = config.triggerDeadzone
                        ).cleanTriggerValues(),
                        right_trigger = TriggerState(
                                value = it.right_trigger.toDouble(),
                                deadzone = config.triggerDeadzone
                        ),
                        leftBumper = previousState.triggerBumperState.leftBumper.updateButtonState(it.left_bumper),
                        rightBumper = previousState.triggerBumperState.rightBumper.updateButtonState(it.right_bumper)
                )
        )

        config.callback
    }

    private val ourGamepad: Gamepad = Gamepad(callback)

    internal fun getGamepad(): Gamepad = ourGamepad

    private var currentState: JoypadState = JoypadState()

    fun changeConfig(newConfig: JoypadConfig) {config = newConfig}

    fun getCurrentState() = currentState
}

data class JoypadState(
        val left_stick: JoystickState = JoystickState(),
        val right_stick: JoystickState = JoystickState(),
        val dPadButtons: DPadState = DPadState(),
        val faceButtons: FaceButtonState = FaceButtonState(),
        val triggerBumperState: TriggerBumperState = TriggerBumperState()
)

interface JoypadCallback {
    open fun callback(joypadState: JoypadState)
}

data class JoystickState(
    val x: Double = 0.0,
    val y: Double = 0.0,
    val deadzone: Double = 0.2,
    val clickyBoi: Boolean = false,
    val active: Boolean = false
) {
    fun cleanValues(): JoystickState {
        val new_x: Double = if (Math.abs(x) < deadzone) {
            0.0
        } else {
            x
        }

        val new_y: Double = if (Math.abs(y) < deadzone) {
            0.0
        } else {
            y
        }

        val new_active: Boolean = (new_x != 0.0) || (new_y != 0.0) || clickyBoi

        return JoystickState(new_x, new_y, deadzone, new_active)
    }
}

enum class ButtonState {
    NOT_PRESSED, PRESSED, HELD;

    fun updateButtonState(button: Boolean): ButtonState {
        when (button) {
            true -> if (this == NOT_PRESSED) {
                return PRESSED
            } else if (this == PRESSED) {
                return HELD
            }
            false -> return NOT_PRESSED
        }
        return HELD
    }
}

data class FaceButtonState(
    val a: ButtonState = ButtonState.NOT_PRESSED,
    val b: ButtonState = ButtonState.NOT_PRESSED,
    val x: ButtonState = ButtonState.NOT_PRESSED,
    val y: ButtonState = ButtonState.NOT_PRESSED
)

data class DPadState(
    val up: ButtonState = ButtonState.NOT_PRESSED,
    val down: ButtonState = ButtonState.NOT_PRESSED,
    val left: ButtonState = ButtonState.NOT_PRESSED,
    val right: ButtonState = ButtonState.NOT_PRESSED
)

data class TriggerState(
    val value: Double = 0.0,
    val deadzone: Double = 0.0
) {
    fun cleanTriggerValues(): TriggerState {
        return TriggerState(value = if (Math.abs(this.value) < deadzone) {
            0.0
        } else {
            this.value
        })
    }
}

data class TriggerBumperState(
    val left_trigger: TriggerState = TriggerState(),
    val right_trigger: TriggerState = TriggerState(),
    val leftBumper: ButtonState = ButtonState.NOT_PRESSED,
    val rightBumper: ButtonState = ButtonState.NOT_PRESSED
)
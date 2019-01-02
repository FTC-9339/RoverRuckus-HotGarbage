package com.roguemafia.rogueftc.control

import com.qualcomm.robotcore.hardware.Gamepad

data class JoypadConfig(
    val joystickDeadzone: Double = 0.2,
    val triggerDeadzone: Double = 0.0
) {
    fun createJoypadFromConfig(joypad: Joypad) {

    }
}

class Joypad(gamepad: Gamepad,
             joypadCallback: JoypadCallback = object : JoypadCallback {
                override fun callback(joypadState: JoypadState) {
                    // Do nothing.
                }
}) {
    private var callback: Gamepad.GamepadCallback = Gamepad.GamepadCallback {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    private val ourGamepad: Gamepad = Gamepad(callback)

    init {
        ourGamepad.copy(gamepad)
    }

    internal fun getGamepad(): Gamepad = ourGamepad

    private var currentState: JoypadState = JoypadState()
}

data class JoypadState(
    val left_stick: JoystickState = JoystickState(),
    val right_stick: JoystickState = JoystickState(),
    val DPadButtons: DPadState = DPadState(),
    val FaceButtons: FaceButtonState = FaceButtonState()
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

        val new_active: Boolean = (new_x != 0.0) || (new_y != 0.0)

        return JoystickState(new_x, new_y, deadzone, new_active)
    }
}

enum class ButtonState {
    NOT_PRESSED, PRESSED, HELD;

    fun updateButtonState(button: Boolean, state: ButtonState): ButtonState {
        when (button) {
            true -> if (state == NOT_PRESSED) {
                return PRESSED
            } else if (state == PRESSED) {
                return HELD
            }
            false -> if (state == PRESSED || state == HELD) return NOT_PRESSED
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
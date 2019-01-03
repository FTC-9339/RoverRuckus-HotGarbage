package org.firstinspires.ftc.teamcode.RogueOpModes.RogueFTC

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import com.roguemafia.rogueftc.control.Joypad
import com.roguemafia.rogueftc.control.JoypadCallback
import com.roguemafia.rogueftc.control.JoypadConfig
import com.roguemafia.rogueftc.control.JoypadState

@TeleOp(name = "Joypad Test", group = "RogueFTC Testing")
class JoypadTest: LinearOpMode() {

    init {
        gamepad1 = Gamepad(Gamepad.GamepadCallback { gamepad ->
            telemetry.addData("""'A' button state: """, gamepad?.a)
            telemetry.addData("""Left joystick y-value: """, gamepad?.left_stick_y)
            telemetry.update()
        })
    }

    override fun runOpMode() {
        /*val joypad = Joypad(config = JoypadConfig(
                callback = object : JoypadCallback {
                    override fun callback(joypadState: JoypadState) {
                        telemetry.addData("A button state: ", joypadState.faceButtons.a.toString())
                        telemetry.addData("Left Joystick State: ", joypadState.left_stick.active)
                        telemetry.addData("Left Trigger State: ", joypadState.triggerBumperState.left_trigger.value)
                        telemetry.update()
                    }
                }
        ), gamepad = gamepad1)
        */

        waitForStart()
        while (opModeIsActive()) {

        }
    }

}
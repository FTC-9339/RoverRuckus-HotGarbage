package com.roguemafia.rogueftc.opmode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.roguemafia.rogueftc.control.Joypad
import com.roguemafia.rogueftc.control.JoypadCallback
import com.roguemafia.rogueftc.control.JoypadState

abstract class TeleOP : LinearOpMode() {
    protected lateinit var joypad1: Joypad
    protected lateinit var joypad2: Joypad

    override fun runOpMode() {
        joypad1 = Joypad(gamepad1, object : JoypadCallback {
            override fun callback(joypadState: JoypadState) {
                joypad1Callback(joypadState)
            }
        })

        joypad2 = Joypad(gamepad2, object : JoypadCallback {
            override fun callback(joypadState: JoypadState) {
                joypad2Callback(joypadState)
            }
        })

        startup()
        waitForStart()
        while (opModeIsActive()) {
            opModeBehavior()
        }
        shutdown()    }

    protected open fun startup() {}

    protected open fun opModeBehavior() {}

    protected open fun shutdown() {}

    protected open fun joypad1Callback(joypadState: JoypadState) {}

    protected open fun joypad2Callback(joypadState: JoypadState) {}
}
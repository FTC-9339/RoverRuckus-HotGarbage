package com.roguemafia.rogueftc.opmode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.roguemafia.rogueftc.control.Joypad
import com.roguemafia.rogueftc.control.JoypadConfig

abstract class TeleOP : LinearOpMode() {
    protected lateinit var joypad1: Joypad
    protected lateinit var joypad2: Joypad

    protected var joypad1Config: JoypadConfig = JoypadConfig()
    protected var joypad2Config: JoypadConfig = JoypadConfig()

    override fun runOpMode() {
        joypad1 = Joypad(joypad1Config, gamepad1)

        joypad2 = Joypad(joypad2Config, gamepad2)

        startup()
        waitForStart()
        while (opModeIsActive()) {
            opModeBehavior()
        }
        shutdown()
    }

    protected abstract fun startup()

    protected abstract fun opModeBehavior()

    protected abstract fun shutdown()
}
package com.roguemafia.rogueftc.robot

import com.qualcomm.robotcore.robot.RobotState
import kotlin.reflect.jvm.internal.impl.load.kotlin.JvmType

interface Subsystem {
    open fun init()

    open fun active(state: RobotState)

    open fun shutdown()
}

data class SubsystemCommand(val commandType: String, val command: JvmType.Object)
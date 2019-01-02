package com.roguemafia.rogueftc.robot.subsystem

import com.qualcomm.robotcore.robot.RobotState

interface Subsystem {
    open fun init()
    open fun active(state: RobotState)
    open fun shutdown()
}
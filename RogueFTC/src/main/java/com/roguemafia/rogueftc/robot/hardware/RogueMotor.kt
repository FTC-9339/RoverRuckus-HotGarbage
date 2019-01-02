package com.roguemafia.rogueftc.robot.hardware

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorController
import com.qualcomm.robotcore.hardware.DcMotorImplEx

class RogueMotor(motor: DcMotor): DcMotorImplEx(motor.controller, motor.portNumber)
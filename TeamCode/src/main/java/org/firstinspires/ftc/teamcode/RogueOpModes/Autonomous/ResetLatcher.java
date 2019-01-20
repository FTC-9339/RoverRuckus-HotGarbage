package org.firstinspires.ftc.teamcode.RogueOpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Reset Latcher", group = "Reset")
public class ResetLatcher extends LinearOpMode {
    public void runOpMode() {
        waitForStart();
        DcMotor Latcher = hardwareMap.dcMotor.get("aux1");
        Latcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Latcher.setTargetPosition(-6300);
        Latcher.setPower(1.0);
        Latcher.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && Latcher.isBusy());
        Latcher.setPower(0.0);
        Latcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}

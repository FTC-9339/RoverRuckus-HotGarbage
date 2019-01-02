package org.firstinspires.ftc.teamcode.RogueOpModes.Manual.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp(name = "BigBoiEncoderTestOP", group = "Testing")
public class BigBoiEncoderTestOP extends LinearOpMode {

    private DcMotor BigBoi = null;

    @Override
    public void runOpMode() {
        BigBoi = hardwareMap.dcMotor.get("BR");
        BigBoi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BigBoi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        while (opModeIsActive()) { BigBoi.setPower(gamepad1.left_stick_y); telemetry.addLine("" + BigBoi.getCurrentPosition()); telemetry.update();}
    }

}

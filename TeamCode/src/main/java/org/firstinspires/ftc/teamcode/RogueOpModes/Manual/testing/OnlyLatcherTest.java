package org.firstinspires.ftc.teamcode.RogueOpModes.Manual.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Latcher Test: No Drive", group = "Testing")
public class OnlyLatcherTest extends LinearOpMode {

    DcMotor Latcher;


    public void runOpMode() {

        Latcher = hardwareMap.dcMotor.get("aux1");
        Latcher.setDirection(DcMotorSimple.Direction.REVERSE);
        Latcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        Latcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while (opModeIsActive()) {

            Latcher.setPower(gamepad1.left_stick_y);


            telemetry.addData("Left Stick Y-value: ", gamepad1.left_stick_y);
            telemetry.addData("Latcher encoder value: ", Latcher.getCurrentPosition());
            telemetry.update();
        }
    }

}

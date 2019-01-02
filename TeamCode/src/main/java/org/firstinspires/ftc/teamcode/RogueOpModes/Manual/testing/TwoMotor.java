package org.firstinspires.ftc.teamcode.RogueOpModes.Manual.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp(name = "Two Motor Test", group = "Testing")
public class TwoMotor extends LinearOpMode {

    public void runOpMode() {
        DcMotor one, two;

        one = hardwareMap.dcMotor.get("mo1");
        two = hardwareMap.dcMotor.get("mo2");

        waitForStart();
        while (opModeIsActive()) {
            one.setPower(gamepad1.left_stick_y);
            two.setPower(gamepad1.right_stick_y);

            telemetry.addData("First motor power: ", one.getPower());
            telemetry.addData("Second motor power: ", two.getPower());
            telemetry.update();
        }
    }

}

package org.firstinspires.ftc.teamcode.RogueOpModes.Manual;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@Disabled
@TeleOp(name = "Main OpMode", group = "Main Drive")
public final class MainOP extends OpMode {

    private DcMotor FL,
                    FR,
                    BL,
                    BR,
                    BigBoi,
                    RopeyBoi,
                    SuccyBoi = null;

    public void init() {
        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BR = hardwareMap.dcMotor.get("BR");

        BigBoi = hardwareMap.dcMotor.get("aux1");
        RopeyBoi = hardwareMap.dcMotor.get("aux2");
        SuccyBoi = hardwareMap.dcMotor.get("aux3");

        telemetry.addLine("Initialized motors successfully.");
        telemetry.update();

        telemetry.addLine("Init phase clear.");
        telemetry.update();
    }

    public void start() {
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);

        BigBoi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RopeyBoi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.clearAll();
    }

    public void loop() {
        telemetry.clearAll();

        FL.setPower(Range.clip(-gamepad1.left_stick_y - gamepad1.left_trigger + gamepad1.right_trigger,-1.0,1.0));
        BL.setPower(Range.clip(-gamepad1.left_stick_y + gamepad1.left_trigger - gamepad1.right_trigger,-1.0,1.0));
        FR.setPower(Range.clip(-gamepad1.right_stick_y + gamepad1.left_trigger - gamepad1.right_trigger,-1.0,1.0));
        BR.setPower(Range.clip(-gamepad1.right_stick_y - gamepad1.left_trigger + gamepad1.right_trigger,-1.0,1.0));

        BigBoi.setPower(gamepad1.dpad_up == true ? 0.5 : (gamepad1.dpad_down == true ? -1.0 : 0.0));
        RopeyBoi.setPower(gamepad1.b == true ? 1.0 : (gamepad1.a == true ? -0.5 : 0.0));
        SuccyBoi.setPower(gamepad1.left_bumper == true ? 1.0 : (gamepad1.right_bumper == true ? -1.0 : 0.0));

        BigBoi.setPower(gamepad1.dpad_up == true ? 0.5 : (gamepad1.dpad_down == true ? -1.0 : 0.0));
        RopeyBoi.setPower(gamepad1.b == true ? 1.0 : (gamepad1.a == true ? -0.5 : 0.0));
        SuccyBoi.setPower(gamepad1.left_bumper == true ? 1.0 : (gamepad1.right_bumper == true ? -1.0 : 0.0));

        telemetry.addLine("FL Motor: " + FL.getPower() + "\n" +
                            "FR Motor: " + FR.getPower() + "\n" +
                            "BL Motor: " + BL.getPower() + "\n" +
                            "BR Motor: " + BR.getPower() + "\n");

        telemetry.update();
    }

}

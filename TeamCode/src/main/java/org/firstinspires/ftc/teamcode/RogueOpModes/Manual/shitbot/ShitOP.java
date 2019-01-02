package org.firstinspires.ftc.teamcode.RogueOpModes.Manual.shitbot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Arrays;

@TeleOp(name = "Rogue OP: Arcade", group = "Testing")
public class ShitOP extends LinearOpMode {

    DcMotor FL,
            FR,
            BL,
            BR;
    DcMotorEx Latcher;

    boolean active = false;
    boolean inverse = false;

    public void runOpMode() {
        FL = hardwareMap.dcMotor.get("FL");
        BL = hardwareMap.dcMotor.get("BL");
        FR = hardwareMap.dcMotor.get("FR");
        BR = hardwareMap.dcMotor.get("BR");

        Latcher = (DcMotorEx) hardwareMap.dcMotor.get("aux1");
        Latcher.setDirection(DcMotorSimple.Direction.REVERSE);
        Latcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Latcher.setMotorDisable();

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.a) {active = !active;}

            if (gamepad1.b) {inverse = !inverse;}

            if (gamepad1.y && !Latcher.isMotorEnabled()) { Latcher.setMotorEnable(); }
            else if (gamepad1.y && Latcher.isMotorEnabled()) { Latcher.setMotorDisable(); }

            if (gamepad1.dpad_up) {Latcher.setPower(1 * ((active) ? 0.5 : 1.0));}
            else if (gamepad1.dpad_down) {Latcher.setPower(-1 * ((active) ? 0.5 : 1.0));}
            else {Latcher.setPower(0);}

            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            FL.setPower(v1);
            FR.setPower(v2);
            BL.setPower(v3);
            BR.setPower(v4);

            telemetry.addData("Speed switch: ", (!active)? "\"I'M FAST AS FRICK BOIIIII\" - Keemstar, 2015"  : "OH LAWD HE COMIN");
            telemetry.addData("Current Motor Velocity: ", ((DcMotorImplEx) FL).getVelocity(AngleUnit.DEGREES));
            telemetry.addData("Latcher encoder value: ", Latcher.getCurrentPosition());
            telemetry.addData("Is Latcher enabled? ", Latcher.isMotorEnabled());
            telemetry.update();
        }
    }

}

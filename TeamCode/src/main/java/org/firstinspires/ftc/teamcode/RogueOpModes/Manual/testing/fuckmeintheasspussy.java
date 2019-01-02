package org.firstinspires.ftc.teamcode.RogueOpModes.Manual.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "fuckmeintheasspussyasshole", group = "Testing")
public class fuckmeintheasspussy extends LinearOpMode {
        DcMotor FL,
                FR,
                BL,
                BR;

        boolean active = false;
        boolean inverse = false;

        public void runOpMode() {
            FL = hardwareMap.dcMotor.get("FL");
            BL = hardwareMap.dcMotor.get("BL");
            FR = hardwareMap.dcMotor.get("FR");
            BR = hardwareMap.dcMotor.get("BR");

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

                FL.setPower(Range.clip((-gamepad1.left_stick_y - gamepad1.left_trigger + gamepad1.right_trigger) * ((active) ? 0.5 : 1.0) * ((inverse) ? -1.0 : 1.0),-1.0,1.0));
                BL.setPower(Range.clip((-gamepad1.left_stick_y + gamepad1.left_trigger - gamepad1.right_trigger) * ((active) ? 0.5 : 1.0) * ((inverse) ? -1.0 : 1.0),-1.0,1.0));
                FR.setPower(Range.clip((-gamepad1.right_stick_y + gamepad1.left_trigger - gamepad1.right_trigger) * ((active) ? 0.5 : 1.0) * ((inverse) ? -1.0 : 1.0),-1.0,1.0));
                BR.setPower(Range.clip((-gamepad1.right_stick_y - gamepad1.left_trigger + gamepad1.right_trigger) * ((active) ? 0.5 : 1.0) * ((inverse) ? -1.0 : 1.0),-1.0,1.0));

                telemetry.addData("Speed switch: ", (!active)? "\"I'M FAST AS FRICK BOIIIII\" - Keemstar, 2015"  : "OH LAWD HE COMIN");
                telemetry.addData("Current Motor Velocity: ", ((DcMotorImplEx) FL).getVelocity(AngleUnit.DEGREES));
                telemetry.update();
            }
        }
}

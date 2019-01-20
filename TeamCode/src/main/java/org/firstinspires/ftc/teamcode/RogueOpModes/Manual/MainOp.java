package org.firstinspires.ftc.teamcode.RogueOpModes.Manual;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Main OpMode", group = "Main")
public class MainOp extends LinearOpMode {

    DcMotor FL,
            FR,
            BL,
            BR,
            Spool;
    DcMotorEx Latcher;

    Servo DiscrimiRotator;
    CRServo Collector;

    boolean active = false;
    boolean inverse = false;
    boolean rotate = false;
    boolean latcher = false;

    ButtonState halfState = ButtonState.NOT_PRESSED;
    ButtonState inverseState = ButtonState.NOT_PRESSED;
    ButtonState rotateState = ButtonState.NOT_PRESSED;
    ButtonState latcherState = ButtonState.NOT_PRESSED;

    public void runOpMode() {
        FL = hardwareMap.dcMotor.get("FL");
        BL = hardwareMap.dcMotor.get("BL");
        FR = hardwareMap.dcMotor.get("FR");
        BR = hardwareMap.dcMotor.get("BR");

        Spool = hardwareMap.dcMotor.get("aux2");
        Spool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Spool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Latcher = (DcMotorEx) hardwareMap.dcMotor.get("aux1");
        Latcher.setDirection(DcMotorSimple.Direction.REVERSE);
        Latcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Latcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Latcher.setMotorDisable();

        DiscrimiRotator = hardwareMap.servo.get("sv1");
        Collector = hardwareMap.crservo.get("sv2");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while (opModeIsActive()) {

            halfState = halfState.updateState(gamepad1.a);
            inverseState = inverseState.updateState(gamepad1.b);
            rotateState = rotateState.updateState(gamepad1.y);
            latcherState = latcherState.updateState(gamepad1.dpad_up);

            if (halfState == ButtonState.PRESSED) active = !active;

            if (inverseState == ButtonState.PRESSED) inverse = !inverse;

            if (rotateState == ButtonState.PRESSED) rotate = !rotate;

            if (latcherState == ButtonState.PRESSED) latcher = !latcher;

            if (gamepad1.dpad_up) {Latcher.setPower(1 * ((active) ? 0.5 : 1.0));}
            else if (gamepad1.dpad_down) {Latcher.setPower(-1 * ((active) ? 0.5 : 1.0));}
            else {Latcher.setPower(0);}

            if (gamepad1.dpad_right) {
                Spool.setPower(1.0 * (active ? 0.5 : 1.0));
            } else if (gamepad1.dpad_left) {
                Spool.setPower(-1.0 * (active ? 0.5 : 1.0));
            } else {
                Spool.setPower(0.0);
            }

            if (gamepad1.left_bumper) {
                Collector.setPower(-1.0 * (active ? 0.5 : 1.0));
            } else if (gamepad1.right_bumper) {
                Collector.setPower(1.0 * (active ? 0.5 : 1.0));
            } else {
                Collector.setPower(0.0);
            }

            Spool.setTargetPosition(Spool.getCurrentPosition());

            DiscrimiRotator.setPosition((rotate) ? 1.0 : -0.7);

            FL.setPower(Range.clip((-gamepad1.left_stick_y - gamepad1.left_trigger + gamepad1.right_trigger) * ((active) ? 0.5 : 1.0) * ((inverse) ? -1.0 : 1.0),-1.0,1.0));
            BL.setPower(Range.clip((-gamepad1.left_stick_y + gamepad1.left_trigger - gamepad1.right_trigger) * ((active) ? 0.5 : 1.0) * ((inverse) ? -1.0 : 1.0),-1.0,1.0));
            FR.setPower(Range.clip((-gamepad1.right_stick_y + gamepad1.left_trigger - gamepad1.right_trigger) * ((active) ? 0.5 : 1.0) * ((inverse) ? -1.0 : 1.0),-1.0,1.0));
            BR.setPower(Range.clip((-gamepad1.right_stick_y - gamepad1.left_trigger + gamepad1.right_trigger) * ((active) ? 0.5 : 1.0) * ((inverse) ? -1.0 : 1.0),-1.0,1.0));

            telemetry.addData("Speed switch: ", (!active)? "\"I'M FAST AS FRICK BOIIIII\" - Keemstar, 2015"  : "OH LAWD HE COMIN");
            telemetry.addData("Current Motor Velocity: ", ((DcMotorImplEx) FL).getVelocity(AngleUnit.DEGREES));
            telemetry.addData("Latcher encoder value: ", Latcher.getCurrentPosition());
            telemetry.addData("Is inversed: ", inverse);
            telemetry.addData("Spool curent value: ", Spool.getCurrentPosition());
            telemetry.update();
        }
    }

    enum ButtonState {
        NOT_PRESSED,
        PRESSED,
        HELD;

        public ButtonState updateState(boolean currentPress) {
            switch (this) {
                case NOT_PRESSED:
                    if (currentPress) return PRESSED;
                    return this;
                case PRESSED:
                    if (currentPress) return HELD;
                    return this;
                case HELD:
                    if (currentPress) return this;
                    default:
                        return NOT_PRESSED;
            }
        }
    }

}

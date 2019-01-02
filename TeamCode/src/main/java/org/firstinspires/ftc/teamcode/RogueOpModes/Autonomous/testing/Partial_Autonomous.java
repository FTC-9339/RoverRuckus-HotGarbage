package org.firstinspires.ftc.teamcode.RogueOpModes.Autonomous.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RogueLibrary.autonomous.AutonomousUtils;

@Autonomous(name = "Main AutoOp", group = "Main")
public class Partial_Autonomous extends LinearOpMode {


    // Encoder Details (Andymark NeveRest Classic 40)
    public static final int amNeverestClassicForty_COUNTS_PER_REVOLUTION = 1120;


    public static final float inWHEEL_CIRCUMFERENCE = (float) (Math.PI * 4);

    // Drive gears/sprocket
    public static final int DRIVE_GEAR = 40;
    public static final int WHEEL_GEAR = 24;

    // Gear Ratio
    public static final double GEAR_RATIO = (double) WHEEL_GEAR / DRIVE_GEAR;

    static final int latcherTgtPos = -6100;


    static final double pow = 0.5;

    DcMotor FL,
            FR,
            BL,
            BR,
            Latcher = null;

    Servo Flipper = null;

    public void runOpMode() {
        FL = hardwareMap.dcMotor.get("FL");
        BL = hardwareMap.dcMotor.get("BL");
        FR = hardwareMap.dcMotor.get("FR");
        BR = hardwareMap.dcMotor.get("BR");

        Latcher = hardwareMap.dcMotor.get("aux1");
        Latcher.setDirection(DcMotorSimple.Direction.REVERSE);
        Latcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Flipper = hardwareMap.servo.get("flip");
        Flipper.setPosition(0.5);

        telemetry.addLine("Ready!");
        waitForStart();

        while (opModeIsActive()) {

            Latcher.setTargetPosition(latcherTgtPos);
            Latcher.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Latcher.setPower(1);

            while (Latcher.isBusy() && !isStopRequested()) ;

            Latcher.setPower(0);
            Latcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            encodedDrive(AutonomousUtils.DRIVE_DIRECTION.STRAFE_RIGHT, 24.0, DistanceUnit.INCH, pow);
            idle();
            encodedDrive(AutonomousUtils.DRIVE_DIRECTION.FORWARD, 36.0, DistanceUnit.INCH, pow);
            idle();

            break;
        }

    }

    public void setRunMode(DcMotor.RunMode runMode) {
        FR.setMode(runMode);
        FL.setMode(runMode);
        BR.setMode(runMode);
        BL.setMode(runMode);
    }

    public void encodedDrive(AutonomousUtils.DRIVE_DIRECTION direction, double distance, DistanceUnit distanceUnit, double power) {
        double speed = Range.clip(Math.abs(power), 0, 1);
        int tgtPos = (int) ((distanceUnit.toInches(distance) / inWHEEL_CIRCUMFERENCE) * GEAR_RATIO) * amNeverestClassicForty_COUNTS_PER_REVOLUTION;

        switch (direction) {
            default:
            case FORWARD:
                FR.setTargetPosition(tgtPos);
                FL.setTargetPosition(tgtPos);
                BR.setTargetPosition(tgtPos);
                BL.setTargetPosition(tgtPos);

                setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

                FR.setPower(speed);
                FL.setPower(speed);
                BR.setPower(speed);
                BL.setPower(speed);

                while (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy() && opModeIsActive()) {
                }

                FR.setPower(0);
                FL.setPower(0);
                BR.setPower(0);
                BL.setPower(0);

                setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                break;

            case BACKWARDS:
                FR.setTargetPosition(-tgtPos);
                FL.setTargetPosition(-tgtPos);
                BR.setTargetPosition(-tgtPos);
                BL.setTargetPosition(-tgtPos);

                setRunMode(DcMotor.RunMode.RUN_TO_POSITION);


                FR.setPower(speed);
                FL.setPower(speed);
                BR.setPower(speed);
                BL.setPower(speed);

                while (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy() && opModeIsActive()) {
                }

                FR.setPower(0);
                FL.setPower(0);
                BR.setPower(0);
                BL.setPower(0);

                setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                break;

            case STRAFE_LEFT:
                FR.setTargetPosition(tgtPos);
                FL.setTargetPosition(-tgtPos);
                BR.setTargetPosition(-tgtPos);
                BL.setTargetPosition(tgtPos);

                setRunMode(DcMotor.RunMode.RUN_TO_POSITION);


                FR.setPower(speed);
                FL.setPower(speed);
                BR.setPower(speed);
                BL.setPower(speed);

                while (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy() && opModeIsActive()) {
                }

                FR.setPower(0);
                FL.setPower(0);
                BR.setPower(0);
                BL.setPower(0);

                setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                break;

            case STRAFE_RIGHT:
                FR.setTargetPosition(-tgtPos);
                FL.setTargetPosition(tgtPos);
                BR.setTargetPosition(tgtPos);
                BL.setTargetPosition(-tgtPos);

                setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

                FR.setPower(speed);
                FL.setPower(speed);
                BR.setPower(speed);
                BL.setPower(speed);

                while (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy() && opModeIsActive()) {
                }

                FR.setPower(0);
                FL.setPower(0);
                BR.setPower(0);
                BL.setPower(0);

                setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                break;
        }

    }
}
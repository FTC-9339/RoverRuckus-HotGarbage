package org.firstinspires.ftc.teamcode.RogueLibrary.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class oldRobot {

    public DcMotorEx FL,
                    FR,
                    BL,
                    BR,
                    BigBoi = null;

    public oldRobot(OpMode opMode) {
        FL = (DcMotorEx) opMode.hardwareMap.dcMotor.get("FL");
        FR = (DcMotorEx) opMode.hardwareMap.dcMotor.get("FR");
        BL = (DcMotorEx) opMode.hardwareMap.dcMotor.get("BL");
        BR = (DcMotorEx) opMode.hardwareMap.dcMotor.get("BR");

        //BigBoi = (DcMotorEx) opMode.hardwareMap.dcMotor.get("aux1");

        opMode.telemetry.addLine("Initialized motors successfully.");
        opMode.telemetry.update();

        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);

        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        opMode.telemetry.addLine("Init phase clear.");
        opMode.telemetry.update();
    }

    public void encodedRotate(double rotationAngleInUnit, AngleUnit rotationAngleUnit, double rotationalSpeed, AngleUnit speedAngleUnit, double power, LinearOpMode opMode) {
        double turnRads = rotationAngleUnit.toRadians(rotationAngleInUnit);
        double turnSpeed = speedAngleUnit.toRadians(rotationalSpeed);

        double turn = turnSpeed/turnRads;

        double flPower = ((1/DistanceUnit.INCH.toMeters(AutonomousUtils.inWHEEL_DIAMETER/2))*(-((AutonomousUtils.inWHEEL_SEPERATION_LENGTH/2)+(AutonomousUtils.inWHEEL_SEPERATION_WIDTH/2)) * turn));
        double frPower = ((1/DistanceUnit.INCH.toMeters(AutonomousUtils.inWHEEL_DIAMETER/2))*(((AutonomousUtils.inWHEEL_SEPERATION_LENGTH/2)+(AutonomousUtils.inWHEEL_SEPERATION_WIDTH/2)) * turn));
        double blPower = ((1/DistanceUnit.INCH.toMeters(AutonomousUtils.inWHEEL_DIAMETER/2))*(-((AutonomousUtils.inWHEEL_SEPERATION_LENGTH/2)+(AutonomousUtils.inWHEEL_SEPERATION_WIDTH/2)) * turn));
        double brPower = ((1/DistanceUnit.INCH.toMeters(AutonomousUtils.inWHEEL_DIAMETER/2))*(((AutonomousUtils.inWHEEL_SEPERATION_LENGTH/2)+(AutonomousUtils.inWHEEL_SEPERATION_WIDTH/2)) * turn));

        int flTarget = (int)(AngleUnit.RADIANS.toDegrees(flPower)/360) * AutonomousUtils.amNeverestClassicFourty_COUNTS_PER_REVOLUTION;
        int frTarget = (int)(AngleUnit.RADIANS.toDegrees(frPower)/360) * AutonomousUtils.amNeverestClassicFourty_COUNTS_PER_REVOLUTION;
        int blTarget = (int)(AngleUnit.RADIANS.toDegrees(blPower)/360) * AutonomousUtils.amNeverestClassicFourty_COUNTS_PER_REVOLUTION;
        int brTarget = (int)(AngleUnit.RADIANS.toDegrees(brPower)/360) * AutonomousUtils.amNeverestClassicFourty_COUNTS_PER_REVOLUTION;

        FL.setTargetPosition(flTarget);
        FR.setTargetPosition(frTarget);
        BL.setTargetPosition(blTarget);
        BR.setTargetPosition(brTarget);

        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        opMode.idle();

        FR.setPower(power);
        FL.setPower(power);
        BR.setPower(power);
        BL.setPower(power);

        while (FL.isBusy() || FR.isBusy() || BL.isBusy() || BR.isBusy() && !opMode.isStopRequested());

        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);

        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        opMode.idle();

    }

    public void encodedDriveAdv(AutonomousUtils.DRIVE_DIRECTION direction, double distanceInUnit, DistanceUnit distanceUnit, double power, LinearOpMode opMode) {
        double distance = distanceUnit.toMeters(distanceInUnit);

        switch (direction) {
            default:
            case FORWARD:

        }
    }

    public void encodedDrive(AutonomousUtils.DRIVE_DIRECTION direction, double distance, DistanceUnit distanceUnit, double power, LinearOpMode opMode) {
        double speed = Range.clip(Math.abs(power), 0, 1);
        int tgtPos = (int) ((distanceUnit.toInches(distance) * AutonomousUtils.inWHEEL_CIRCUMFERENCE) * AutonomousUtils.GEAR_RATIO) * AutonomousUtils.amNeverestClassicFourty_COUNTS_PER_REVOLUTION;

        switch (direction) {
            default:
            case FORWARD:
                FR.setTargetPosition(-tgtPos);
                FL.setTargetPosition(-tgtPos);
                BR.setTargetPosition(-tgtPos);
                BL.setTargetPosition(-tgtPos);

                setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

                opMode.idle();

                FR.setPower(speed);
                FL.setPower(speed);
                BR.setPower(speed);
                BL.setPower(speed);

                while (FL.isBusy() || FR.isBusy() || BL.isBusy() || BR.isBusy() && !opMode.isStopRequested());

                FR.setPower(0);
                FL.setPower(0);
                BR.setPower(0);
                BL.setPower(0);

                setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                opMode.idle();

                break;

            case BACKWARDS:
                FR.setTargetPosition(tgtPos);
                FL.setTargetPosition(tgtPos);
                BR.setTargetPosition(tgtPos);
                BL.setTargetPosition(tgtPos);

                setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

                opMode.idle();

                FR.setPower(speed);
                FL.setPower(speed);
                BR.setPower(speed);
                BL.setPower(speed);

                while (FL.isBusy() || FR.isBusy() || BL.isBusy() || BR.isBusy() && !opMode.isStopRequested());

                FR.setPower(0);
                FL.setPower(0);
                BR.setPower(0);
                BL.setPower(0);

                setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                opMode.idle();

                break;

            case STRAFE_LEFT:
                FR.setTargetPosition(tgtPos);
                FL.setTargetPosition(-tgtPos);
                BR.setTargetPosition(-tgtPos);
                BL.setTargetPosition(tgtPos);

                setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

                opMode.idle();

                FR.setPower(speed);
                FL.setPower(speed);
                BR.setPower(speed);
                BL.setPower(speed);

                while (FL.isBusy() || FR.isBusy() || BL.isBusy() || BR.isBusy() && !opMode.isStopRequested());

                FR.setPower(0);
                FL.setPower(0);
                BR.setPower(0);
                BL.setPower(0);

                setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                opMode.idle();

                break;

            case STRAFE_RIGHT:
                FR.setTargetPosition(-tgtPos);
                FL.setTargetPosition(tgtPos);
                BR.setTargetPosition(tgtPos);
                BL.setTargetPosition(-tgtPos);

                setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

                opMode.idle();

                FR.setPower(speed);
                FL.setPower(speed);
                BR.setPower(speed);
                BL.setPower(speed);

                while (FL.isBusy() || FR.isBusy() || BL.isBusy() || BR.isBusy() && !opMode.isStopRequested());

                FR.setPower(0);
                FL.setPower(0);
                BR.setPower(0);
                BL.setPower(0);

                setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                opMode.idle();

                break;
        }
    }

    public void setRunMode(DcMotor.RunMode runMode) {
        FR.setMode(runMode);
        FL.setMode(runMode);
        BR.setMode(runMode);
        BL.setMode(runMode);
    }

}

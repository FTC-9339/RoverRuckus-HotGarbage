package org.firstinspires.ftc.teamcode.RogueOpModes.Autonomous.testing;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RogueLibrary.autonomous.AutonomousUtils;

@Autonomous(name = "Rotational Drive Test", group = "Testing")
public class DirectionalTest extends LinearOpMode {

    // Encoder Details (Andymark NeveRest Classic 40)
    public static final int     amNeverestClassicFourty_COUNTS_PER_REVOLUTION = 1120;

    public static final double  COUNTS_PER_INCH = amNeverestClassicFourty_COUNTS_PER_REVOLUTION/AutonomousUtils.inWHEEL_CIRCUMFERENCE;


    DcMotorEx FL,
            FR,
            BL,
            BR;

    public void runOpMode() {
        FL = (DcMotorEx) hardwareMap.dcMotor.get("FL");
        FR = (DcMotorEx) hardwareMap.dcMotor.get("FR");
        BL = (DcMotorEx) hardwareMap.dcMotor.get("BL");
        BR = (DcMotorEx) hardwareMap.dcMotor.get("BR");

        telemetry.addLine("Initialized motors successfully.");
        telemetry.update();

        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);

        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addLine("Init phase clear.");
        telemetry.update();
        waitForStart();
            telemetry.clearAll();
            telemetry.addLine("Going FORWARD!");
            telemetry.update();
            encodedDrive(new Velocity(1.0, 0.0, 0.0, DistanceUnit.METER, AngleUnit.RADIANS), 12.0, DistanceUnit.INCH);
            encodedDrive(new Velocity(1.0, 0.0, 0.0, DistanceUnit.METER, AngleUnit.RADIANS), 12.0, DistanceUnit.INCH);
            encodedTurn(360, AngleUnit.DEGREES, 0.5 * Math.PI, AngleUnit.RADIANS);
    }

    public void setRunMode(DcMotor.RunMode runMode) {
        FR.setMode(runMode);
        FL.setMode(runMode);
        BR.setMode(runMode);
        BL.setMode(runMode);
        idle();
    }

    public void setTarget(int FLtgtPos, int FRtgtPos, int BLtgtPos, int BRtgtPos) {
        FL.setTargetPosition(FL.getCurrentPosition() + FLtgtPos);
        FR.setTargetPosition(FR.getCurrentPosition() + FRtgtPos);
        BL.setTargetPosition(BL.getCurrentPosition() + BLtgtPos);
        BR.setTargetPosition(BR.getCurrentPosition() + BRtgtPos);
    }

    public void setPower(double power) {
        FL.setPower(power);
        FR.setPower(power);
        BL.setPower(power);
        BR.setPower(power);
    }

    private void encodedTurn(double rotation, AngleUnit rotationUnit, double rotationalVelocity, AngleUnit velocityUnit) {
        double FLvelocity = -(DistanceUnit.INCH.toMeters(AutonomousUtils.inWHEEL_SEPERATION_LENGTH + AutonomousUtils.inWHEEL_SEPERATION_WIDTH) * velocityUnit.toRadians(rotationalVelocity))*(1/DistanceUnit.INCH.toMeters(2.0)) * 0.6;
        double FRvelocity = (DistanceUnit.INCH.toMeters(AutonomousUtils.inWHEEL_SEPERATION_LENGTH + AutonomousUtils.inWHEEL_SEPERATION_WIDTH) * velocityUnit.toRadians(rotationalVelocity))*(1/DistanceUnit.INCH.toMeters(2.0)) * 0.6;
        double BLvelocity = -(DistanceUnit.INCH.toMeters(AutonomousUtils.inWHEEL_SEPERATION_LENGTH + AutonomousUtils.inWHEEL_SEPERATION_WIDTH) * velocityUnit.toRadians(rotationalVelocity))*(1/DistanceUnit.INCH.toMeters(2.0)) * 0.6;
        double BRvelocity = (DistanceUnit.INCH.toMeters(AutonomousUtils.inWHEEL_SEPERATION_LENGTH + AutonomousUtils.inWHEEL_SEPERATION_WIDTH) * velocityUnit.toRadians(rotationalVelocity))*(1/DistanceUnit.INCH.toMeters(2.0)) * 0.6;

        Log.i("FLvelocity", String.valueOf(FLvelocity));
        telemetry.addData("FLvelocity: ", String.valueOf(FLvelocity));
        telemetry.update();

        double time = rotationUnit.toRadians(rotation) / velocityUnit.toRadians(rotationalVelocity);

        int FLtarget = (int) (DistanceUnit.METER.toInches((FLvelocity * 2.0 * time)) * COUNTS_PER_INCH);
        int FRtarget = (int) (DistanceUnit.METER.toInches((FRvelocity * 2.0 * time)) * COUNTS_PER_INCH);
        int BLtarget = (int) (DistanceUnit.METER.toInches((BLvelocity * 2.0 * time)) * COUNTS_PER_INCH);
        int BRtarget = (int) (DistanceUnit.METER.toInches((BRvelocity * 2.0 * time)) * COUNTS_PER_INCH);

        Log.i("FL Target Position", String.valueOf(FLtarget));
        Log.i("Distance to travel", String.valueOf(FLtarget/COUNTS_PER_INCH));

        telemetry.addLine("Distance to travel: " + String.valueOf(FLtarget/COUNTS_PER_INCH));
        telemetry.update();

        setTarget(FLtarget, FRtarget, BLtarget, BRtarget);
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setVelocity(FLvelocity, AngleUnit.RADIANS);
        FR.setVelocity(FRvelocity, AngleUnit.RADIANS);
        BL.setVelocity(BLvelocity, AngleUnit.RADIANS);
        BR.setVelocity(BRvelocity, AngleUnit.RADIANS);
        while (opModeIsActive() && FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy());
        FL.setVelocity(0.0);
        FR.setVelocity(0.0);
        BL.setVelocity(0.0);
        BR.setVelocity(0.0);
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void encodedDrive(Velocity robotVelocity, double distance, DistanceUnit distanceUnit) {
        double FLvelocity = (robotVelocity.velocityUnit.toMeters(robotVelocity.x) - robotVelocity.velocityUnit.toMeters(robotVelocity.y))*(1/DistanceUnit.INCH.toMeters(2.0)) * 0.6;
        double FRvelocity = (robotVelocity.velocityUnit.toMeters(robotVelocity.x) + robotVelocity.velocityUnit.toMeters(robotVelocity.y))*(1/DistanceUnit.INCH.toMeters(2.0)) * 0.6;
        double BLvelocity = (robotVelocity.velocityUnit.toMeters(robotVelocity.x) + robotVelocity.velocityUnit.toMeters(robotVelocity.y))*(1/DistanceUnit.INCH.toMeters(2.0)) * 0.6;
        double BRvelocity = (robotVelocity.velocityUnit.toMeters(robotVelocity.x) - robotVelocity.velocityUnit.toMeters(robotVelocity.y))*(1/DistanceUnit.INCH.toMeters(2.0)) * 0.6;

        Log.i("FLvelocity", String.valueOf(FLvelocity));

        double Xtime = (Double.isInfinite(distanceUnit.toMeters(distance) / robotVelocity.velocityUnit.toMeters(robotVelocity.x))) ? 0.0 : distanceUnit.toMeters(distance) / robotVelocity.velocityUnit.toMeters(robotVelocity.x);
        double Ytime = (Double.isInfinite(distanceUnit.toMeters(distance) / robotVelocity.velocityUnit.toMeters(robotVelocity.y))) ? 0.0 : distanceUnit.toMeters(distance) / robotVelocity.velocityUnit.toMeters(robotVelocity.y);

        double time = (Xtime+Ytime)/2;

        Log.i("Time", String.valueOf(time));

        int FLtarget = (int) (DistanceUnit.METER.toInches((FLvelocity * 2.0 * time)) * COUNTS_PER_INCH);
        int FRtarget = (int) (DistanceUnit.METER.toInches((FRvelocity * 2.0 * time)) * COUNTS_PER_INCH);
        int BLtarget = (int) (DistanceUnit.METER.toInches((BLvelocity * 2.0 * time)) * COUNTS_PER_INCH);
        int BRtarget = (int) (DistanceUnit.METER.toInches((BRvelocity * 2.0 * time)) * COUNTS_PER_INCH);

        Log.i("FL Target Position", String.valueOf(FLtarget));
        Log.i("Distance to travel", String.valueOf(FLtarget/COUNTS_PER_INCH * 0.6));

        telemetry.addLine("Distance to travel: " + String.valueOf(FLtarget/COUNTS_PER_INCH * 0.6));
        telemetry.update();

        setTarget(FLtarget, FRtarget, BLtarget, BRtarget);
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setVelocity(FLvelocity, AngleUnit.RADIANS);
        FR.setVelocity(FRvelocity, AngleUnit.RADIANS);
        BL.setVelocity(BLvelocity, AngleUnit.RADIANS);
        BR.setVelocity(BRvelocity, AngleUnit.RADIANS);
        while (opModeIsActive() && FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy());
        FL.setVelocity(0.0);
        FR.setVelocity(0.0);
        BL.setVelocity(0.0);
        BR.setVelocity(0.0);
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    class Velocity {
        public final double x, y, z;
        public final DistanceUnit velocityUnit;
        public final AngleUnit rotationalUnit;

        public Velocity(double x, double y, double z, DistanceUnit velocityUnit, AngleUnit rotationalUnit) {
            this.x = x;
            this.y = y;
            this.z = z;
            this.velocityUnit = velocityUnit;
            this.rotationalUnit = rotationalUnit;
        }
    }

}

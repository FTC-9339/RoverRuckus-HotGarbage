package org.firstinspires.ftc.teamcode.RogueOpModes.Autonomous;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.RogueLibrary.autonomous.AutonomousUtils;
import org.firstinspires.ftc.teamcode.RogueLibrary.vision.managers.TensorflowManager;

public class DepotSideOP extends LinearOpMode {

    // Encoder Details (Andymark NeveRest Classic 40)
    public static final int     amNeverestClassicFourty_COUNTS_PER_REVOLUTION = 1120;

    public static final double  COUNTS_PER_INCH = amNeverestClassicFourty_COUNTS_PER_REVOLUTION/AutonomousUtils.inWHEEL_CIRCUMFERENCE;

    public static final float   inWHEEL_SEPERATION_WIDTH = 13;
    public static final float   inWHEEL_SEPERATION_LENGTH = 11.5f;

    public static final float   inWHEEL_RADIUS = 2.0f;

    static final int latcherTgtPos = -6850;

    DcMotorEx FL,
            FR,
            BL,
            BR;
    DcMotor        Latcher;

    TensorflowManager tfod;

    Servo marker;

    private SampleState sampleState;

    public void runOpMode() {
        FL = (DcMotorEx) hardwareMap.dcMotor.get("FL");
        FR = (DcMotorEx) hardwareMap.dcMotor.get("FR");
        BL = (DcMotorEx) hardwareMap.dcMotor.get("BL");
        BR = (DcMotorEx) hardwareMap.dcMotor.get("BR");
        tfod = new TensorflowManager(this, true);

        Latcher = hardwareMap.dcMotor.get("aux1");
        Latcher.setDirection(DcMotorSimple.Direction.REVERSE);
        Latcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addLine("Initialized motors successfully.");
        telemetry.update();

        marker = hardwareMap.servo.get("sv1");
        marker.setPosition(-1.0);

        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);

        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addLine("Init phase clear.");
        telemetry.update();

        waitForStart();

        tfod.startDetection();

        telemetry.clearAll();

        Latcher.setTargetPosition(latcherTgtPos);
        Latcher.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Latcher.setPower(1);

        while (Latcher.isBusy() && opModeIsActive()) ;

        Latcher.setPower(0);
        Latcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        encodedDrive(new Velocity(0.0, 1.0, 0.0, DistanceUnit.INCH, UnnormalizedAngleUnit.RADIANS), -3, DistanceUnit.INCH);
        encodedDrive(new Velocity(1.0, 0.0, 0.0, DistanceUnit.INCH, UnnormalizedAngleUnit.RADIANS), 3, DistanceUnit.INCH);
        encodedDrive(new Velocity(0.0, 1.0, 0.0, DistanceUnit.INCH, UnnormalizedAngleUnit.RADIANS), 3, DistanceUnit.INCH);
        encodedDrive(new Velocity(6.0, 0.0,0.0, DistanceUnit.INCH, UnnormalizedAngleUnit.DEGREES), 21 - 11.25, DistanceUnit.INCH);

        sleep(1500);

        if (tfod.goldMineralInView()) {
            encodedDrive(new Velocity(6.0, 0.0, 0.0, DistanceUnit.INCH, UnnormalizedAngleUnit.DEGREES), 15, DistanceUnit.INCH);
            sampleState = SampleState.CENTER;
        } else {
            encodedTurn(new Velocity(0.0,0.0,45.0, DistanceUnit.INCH, UnnormalizedAngleUnit.DEGREES), -45, UnnormalizedAngleUnit.DEGREES);
            sleep(500);
            if (tfod.goldMineralInView()) {
                encodedDrive(new Velocity(6.0, 0.0, 0.0, DistanceUnit.INCH, UnnormalizedAngleUnit.DEGREES), 15, DistanceUnit.INCH);
                sampleState = SampleState.LEFT;
            } else {
                encodedTurn(new Velocity(0.0,0.0,45.0, DistanceUnit.INCH, UnnormalizedAngleUnit.DEGREES), 90, UnnormalizedAngleUnit.DEGREES);
                encodedDrive(new Velocity(6.0, 0.0, 0.0, DistanceUnit.INCH, UnnormalizedAngleUnit.DEGREES), 15, DistanceUnit.INCH);
                sampleState = SampleState.RIGHT;
            }
        }

        switch (sampleState) {
            case CENTER:
                encodedDrive(new Velocity(6.0, 0.0, 0.0, DistanceUnit.INCH, UnnormalizedAngleUnit.DEGREES), -12, DistanceUnit.INCH);
                break;
            case LEFT:
                encodedDrive(new Velocity(6.0, 0.0, 0.0, DistanceUnit.INCH, UnnormalizedAngleUnit.DEGREES), -15, DistanceUnit.INCH);
                encodedTurn(new Velocity(0.0,0.0,45.0, DistanceUnit.INCH, UnnormalizedAngleUnit.DEGREES), 45, UnnormalizedAngleUnit.DEGREES);
                break;
            case RIGHT:
                encodedDrive(new Velocity(6.0, 0.0, 0.0, DistanceUnit.INCH, UnnormalizedAngleUnit.DEGREES), -15, DistanceUnit.INCH);
                encodedTurn(new Velocity(0.0,0.0,45.0, DistanceUnit.INCH, UnnormalizedAngleUnit.DEGREES), -45, UnnormalizedAngleUnit.DEGREES);
                break;
        }

        encodedDrive(new Velocity(6.0, 0.0, 0.0, DistanceUnit.INCH, UnnormalizedAngleUnit.RADIANS), 20, DistanceUnit.INCH);
        encodedTurn(new Velocity(0.0,0.0,45.0, DistanceUnit.INCH, UnnormalizedAngleUnit.DEGREES), 180, UnnormalizedAngleUnit.DEGREES);
        marker.setPosition(1.0);
        while (marker.getPosition() != 1.0 && opModeIsActive());
        encodedTurn(new Velocity(0.0,0.0,45.0, DistanceUnit.INCH, UnnormalizedAngleUnit.DEGREES), -45, UnnormalizedAngleUnit.DEGREES);
        encodedDrive(new Velocity(12.0,0.0,0.0, DistanceUnit.INCH, UnnormalizedAngleUnit.DEGREES), 70, DistanceUnit.INCH);
        tfod.shutdown();
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

    private void encodedTurn(final Velocity velocity, final double targetAngle, final UnnormalizedAngleUnit targetUnit) {
        double yo = ((inWHEEL_SEPERATION_LENGTH+inWHEEL_SEPERATION_WIDTH)/2);

        double FLvelocity = ((-yo * velocity.rotationalUnit.toRadians(velocity.z)/inWHEEL_RADIUS) *0.6);
        double FRvelocity = ((yo * velocity.rotationalUnit.toRadians(velocity.z)/inWHEEL_RADIUS) *0.6);
        double BLvelocity = ((-yo * velocity.rotationalUnit.toRadians(velocity.z)/inWHEEL_RADIUS) *0.6);
        double BRvelocity = ((yo * velocity.rotationalUnit.toRadians(velocity.z)/inWHEEL_RADIUS) *0.6);

        Log.i("FLvelocity", String.valueOf(FLvelocity));
        telemetry.addData("FLvelocity: ", String.valueOf(FLvelocity));
        telemetry.update();

        double time = targetUnit.toDegrees(targetAngle) / velocity.rotationalUnit.toDegrees(velocity.z);

        if (Double.isInfinite(time)) {
            time = 0.0;
        }

        int FLtarget = (int) ((FLvelocity * time) * COUNTS_PER_INCH * inWHEEL_RADIUS);
        int FRtarget = (int) ((FRvelocity * time) * COUNTS_PER_INCH * inWHEEL_RADIUS);
        int BLtarget = (int) ((BLvelocity * time) * COUNTS_PER_INCH * inWHEEL_RADIUS);
        int BRtarget = (int) ((BRvelocity * time) * COUNTS_PER_INCH * inWHEEL_RADIUS);

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
        while (opModeIsActive() && FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy()) {
            telemetry.addData("FLMotor current pos: ", FL.getCurrentPosition());
            telemetry.addData("FLMotor Target Position: ", FLtarget);
            telemetry.update();
        }
        FL.setVelocity(0.0);
        FR.setVelocity(0.0);
        BL.setVelocity(0.0);
        BR.setVelocity(0.0);
    }

    private void encodedDrive(final Velocity robotVelocity, final double distance, final DistanceUnit distanceUnit) {

        double FLvelocity = (robotVelocity.velocityUnit.toMeters(-robotVelocity.x) - robotVelocity.velocityUnit.toMeters(robotVelocity.y))/DistanceUnit.INCH.toMeters(inWHEEL_RADIUS) * 0.6;
        double FRvelocity = (robotVelocity.velocityUnit.toMeters(-robotVelocity.x) + robotVelocity.velocityUnit.toMeters(robotVelocity.y))/DistanceUnit.INCH.toMeters(inWHEEL_RADIUS) * 0.6;
        double BLvelocity = (robotVelocity.velocityUnit.toMeters(-robotVelocity.x) + robotVelocity.velocityUnit.toMeters(robotVelocity.y))/DistanceUnit.INCH.toMeters(inWHEEL_RADIUS) * 0.6;
        double BRvelocity = (robotVelocity.velocityUnit.toMeters(-robotVelocity.x) - robotVelocity.velocityUnit.toMeters(robotVelocity.y))/DistanceUnit.INCH.toMeters(inWHEEL_RADIUS) * 0.6;

        Log.i("FLvelocity", String.valueOf(FLvelocity));

        double Xtime = (Double.isInfinite(distanceUnit.toMeters(distance) / robotVelocity.velocityUnit.toMeters(robotVelocity.x))) ? 0.0 : distanceUnit.toMeters(distance) / robotVelocity.velocityUnit.toMeters(robotVelocity.x);
        double Ytime = (Double.isInfinite(distanceUnit.toMeters(distance) / robotVelocity.velocityUnit.toMeters(robotVelocity.y))) ? 0.0 : distanceUnit.toMeters(distance) / robotVelocity.velocityUnit.toMeters(robotVelocity.y);

        double time = 0.0;

        if (Xtime != 0.0 && Ytime == 0.0) {
            time = Xtime;
        } else if (Ytime != 0.0 && Xtime == 0.0) {
            time = Ytime;
        } else {
            time += Xtime + Ytime;
            time = time/2;
        }

        Log.i("Time", String.valueOf(time));

        int FLtarget = (int) ((FLvelocity * time) * COUNTS_PER_INCH * inWHEEL_RADIUS);
        int FRtarget = (int) ((FRvelocity * time) * COUNTS_PER_INCH * inWHEEL_RADIUS);
        int BLtarget = (int) ((BLvelocity * time) * COUNTS_PER_INCH * inWHEEL_RADIUS);
        int BRtarget = (int) ((BRvelocity * time) * COUNTS_PER_INCH * inWHEEL_RADIUS);

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
        while (opModeIsActive() && FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy()) {
            telemetry.addData("FLMotor current pos: ", FL.getCurrentPosition());
            telemetry.addData("FLMotor Target Position: ", FLtarget);
            telemetry.update();
        }
        FL.setVelocity(0.0);
        FR.setVelocity(0.0);
        BL.setVelocity(0.0);
        BR.setVelocity(0.0);
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    class Velocity {
        public final double x, y, z;
        public final DistanceUnit velocityUnit;
        public final UnnormalizedAngleUnit rotationalUnit;

        public Velocity(double x, double y, double z, DistanceUnit velocityUnit, UnnormalizedAngleUnit rotationalUnit) {
            this.x = x;
            this.y = y;
            this.z = z;
            this.velocityUnit = velocityUnit;
            this.rotationalUnit = rotationalUnit;
        }

        public Velocity toUnit(DistanceUnit newVelocityUnit, UnnormalizedAngleUnit newRotationalUnit) {
            return new Velocity(
                    newVelocityUnit.fromUnit(velocityUnit, x),
                    newVelocityUnit.fromUnit(velocityUnit, y),
                    newRotationalUnit.fromUnit(rotationalUnit, z),
                    newVelocityUnit,
                    newRotationalUnit
            );
        }
    }

    enum SampleState {
        LEFT,
        CENTER,
        RIGHT
    }

}

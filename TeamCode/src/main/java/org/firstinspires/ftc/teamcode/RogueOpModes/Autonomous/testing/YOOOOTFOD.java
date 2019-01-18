package org.firstinspires.ftc.teamcode.RogueOpModes.Autonomous.testing;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.RogueLibrary.autonomous.AutonomousUtils;
import org.firstinspires.ftc.teamcode.RogueLibrary.vision.managers.TensorflowManager;

@Autonomous(name = "YOOO TFOD", group = "Testing")
public class YOOOOTFOD extends LinearOpMode {

    // Encoder Details (Andymark NeveRest Classic 40)
    public static final int     amNeverestClassicFourty_COUNTS_PER_REVOLUTION = 1120;

    public static final double  COUNTS_PER_INCH = amNeverestClassicFourty_COUNTS_PER_REVOLUTION/AutonomousUtils.inWHEEL_CIRCUMFERENCE;

    public static final float   inWHEEL_SEPERATION_WIDTH = 13;
    public static final float   inWHEEL_SEPERATION_LENGTH = 11.5f;

    public static final float   inWHEEL_RADIUS = 1.8f;

    DcMotorEx FL,
            FR,
            BL,
            BR;

    TensorflowManager tfod;

    BNO055IMU imu;
    PIDController pidRotate = new PIDController(0.05, 0.0, 0.0);

    Orientation             lastAngles = new Orientation();
    double                  globalAngle;

    public void runOpMode() {
        FL = (DcMotorEx) hardwareMap.dcMotor.get("FL");
        FR = (DcMotorEx) hardwareMap.dcMotor.get("FR");
        BL = (DcMotorEx) hardwareMap.dcMotor.get("BL");
        BR = (DcMotorEx) hardwareMap.dcMotor.get("BR");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        tfod = new TensorflowManager(this, true);

        telemetry.addLine("Initialized motors successfully.");
        telemetry.update();

        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);

        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addLine("Init phase clear.");
        telemetry.update();

        telemetry.addLine("CALIBRATING GYRO");
        telemetry.update();

        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addLine("CALIBRATED");
        telemetry.update();

        waitForStart();

        tfod.startDetection();

        telemetry.clearAll();

        sleep(1000);

        if (tfod.goldMineralInView()) {
            encodedDrive(new Velocity(12.0, 0.0, 0.0, DistanceUnit.INCH, UnnormalizedAngleUnit.DEGREES), 30, DistanceUnit.INCH);
        } else {
            //rotate(-45, 0.5);
            encodedTurn(new Velocity(0.0,0.0,90.0, DistanceUnit.INCH, UnnormalizedAngleUnit.DEGREES), -45, UnnormalizedAngleUnit.DEGREES);
            sleep(250);
            if (tfod.goldMineralInView()) {
                encodedDrive(new Velocity(12.0, 0.0, 0.0, DistanceUnit.INCH, UnnormalizedAngleUnit.DEGREES), 24, DistanceUnit.INCH);
            } else {
                //rotate(90, 0.5);
                encodedTurn(new Velocity(0.0,0.0,90.0, DistanceUnit.INCH, UnnormalizedAngleUnit.DEGREES), 90, UnnormalizedAngleUnit.DEGREES);
                encodedDrive(new Velocity(12.0, 0.0, 0.0, DistanceUnit.INCH, UnnormalizedAngleUnit.DEGREES), 24, DistanceUnit.INCH);
            }
        }

        tfod.shutdown();
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        // restart imu angle tracking.
        resetAngle();

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle with a minimum of 20%.
        // This is to prevent the robots momentum from overshooting the turn after we turn off the
        // power. The PID controller reports onTarget() = true when the difference between turn
        // angle and target angle is within 2% of target (tolerance). This helps prevent overshoot.
        // The minimum power is determined by testing and must enough to prevent motor stall and
        // complete the turn. Note: if the gap between the starting power and the stall (minimum)
        // power is small, overshoot may still occur. Overshoot is dependant on the motor and
        // gearing configuration, starting power, weight of the robot and the on target tolerance.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, 90);
        pidRotate.setOutputRange(.20, power);
        pidRotate.setTolerance(2);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0)
            {
                FL.setPower(-power);
                FR.setPower(power);
                BL.setPower(-power);
                BR.setPower(power);
                sleep(100);
            }

            do
            {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                FL.setPower(-power);
                FR.setPower(power);
                BL.setPower(-power);
                BR.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                FL.setPower(-power);
                FR.setPower(power);
                BL.setPower(-power);
                BR.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        setPower(0.0);

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
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

    private void gyroTurn() {

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

        double FLvelocity = (robotVelocity.velocityUnit.toMeters(robotVelocity.x) - robotVelocity.velocityUnit.toMeters(robotVelocity.y))/DistanceUnit.INCH.toMeters(inWHEEL_RADIUS) * 0.6;
        double FRvelocity = (robotVelocity.velocityUnit.toMeters(robotVelocity.x) + robotVelocity.velocityUnit.toMeters(robotVelocity.y))/DistanceUnit.INCH.toMeters(inWHEEL_RADIUS) * 0.6;
        double BLvelocity = (robotVelocity.velocityUnit.toMeters(robotVelocity.x) + robotVelocity.velocityUnit.toMeters(robotVelocity.y))/DistanceUnit.INCH.toMeters(inWHEEL_RADIUS) * 0.6;
        double BRvelocity = (robotVelocity.velocityUnit.toMeters(robotVelocity.x) - robotVelocity.velocityUnit.toMeters(robotVelocity.y))/DistanceUnit.INCH.toMeters(inWHEEL_RADIUS) * 0.6;

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

}

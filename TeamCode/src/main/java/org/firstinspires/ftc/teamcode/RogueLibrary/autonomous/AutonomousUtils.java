package org.firstinspires.ftc.teamcode.RogueLibrary.autonomous;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

public final class AutonomousUtils {

    public static final float   mm_PER_INCH = 25.4f;

    // Field measurements (NOTE: the field is a square, these measurements apply to all sides of the field)
    public static final int     inFIELD_WIDTH = 72;
    public static final float   mmFIELD_WIDTH = inFIELD_WIDTH * mm_PER_INCH;
    public static final float   mmNAV_TARGET_HEIGHT = 6 * mm_PER_INCH;

    // IWheel measurements (VEX Mecanum IWheel 4")
    public static final int     inWHEEL_DIAMETER = 4;
    public static final float   inWHEEL_CIRCUMFERENCE = (float) (Math.PI * inWHEEL_DIAMETER);

    // Drive gears/sprocket
    public static final int DRIVE_GEAR = 40;
    public static final int WHEEL_GEAR = 24;

    // Gear Ratio
    public static final double GEAR_RATIO = (double) WHEEL_GEAR/DRIVE_GEAR;

    public static final double DRIVE_MAX_RADIANS_PER_SEC = 0.0;

    // Encoder Details (US Digital E4P)
    public static final int     usDigitalEncoder_COUNTS_PER_REVOLUTION = 360;

    // Encoder Details (Andymark NeveRest Classic 40)
    public static final int     amNeverestClassicFourty_COUNTS_PER_REVOLUTION = 1120;

    // oldRobot measurements
    public static final float   inROBOT_WIDTH = 16.5f;
    public static final float   inWHEEL_SEPERATION_WIDTH = 13;
    public static final float   inWHEEL_SEPERATION_LENGTH = 11.5f;

    // Camera measurements
    // What camera we are using
    public static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    // How far forward the center of the camera is from the center of the robot.
    public static final float   inCAMERA_FORWARD_DISPLACEMENT = (float) 7.0;
    // How far up the center of the camera is from the ground.
    public static final float   inCAMERA_VERTICAL_DISPLACEMENT = (float) 11 + (6/8);
    // How far left horizontally the camera is from the center of the robot.
    public static final float   inCAMERA_LEFT_DISPLACEMENT = 7.5f;
    // Rotation (in degrees) of the camera on the Y-axis (assuming y-axis is up in Vuforia) from the center of the robot.
    // Assumes 0 degrees is facing to the LEFT of the robot.
    public static final float   degCAMERA_ROTATIONAL_DISPLACEMENT = CAMERA_CHOICE == FRONT ? 90 : -90;
    // All above measurements in MM
    public static final float   mmCAMERA_FORWARD_DISPLACEMENT = inCAMERA_FORWARD_DISPLACEMENT * mm_PER_INCH;
    public static final float   mmCAMERA_VERTICAL_DISPLACEMENT = inCAMERA_VERTICAL_DISPLACEMENT * mm_PER_INCH;
    public static final float   mmCAMERA_LEFT_DISPLACEMENT = inCAMERA_LEFT_DISPLACEMENT * mm_PER_INCH;

    // TensorFlow stuff
    public static final String  TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    public static final String  LABEL_GOLD_MINERAL = "Gold Mineral";
    public static final String  LABEL_SILVER_MINERAL = "Silver Mineral";

    // Vuforia stuff
    public static final OpenGLMatrix PHONE_LOCATION_ON_ROBOT = OpenGLMatrix
            .translation(mmCAMERA_FORWARD_DISPLACEMENT, mmCAMERA_LEFT_DISPLACEMENT, mmCAMERA_VERTICAL_DISPLACEMENT)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                    degCAMERA_ROTATIONAL_DISPLACEMENT, 0, 90));

    // Vuforia Navigation Matrices
    public static final OpenGLMatrix BLUE_NAVMARK_ON_FIELD = OpenGLMatrix
            .translation(0, mmFIELD_WIDTH, mmNAV_TARGET_HEIGHT)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
    public static final OpenGLMatrix RED_NAVMARK_ON_FIELD = OpenGLMatrix
            .translation(0, -mmFIELD_WIDTH, mmNAV_TARGET_HEIGHT)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
    public static final OpenGLMatrix FRONT_NAVMARK_ON_FIELD = OpenGLMatrix
            .translation(-mmFIELD_WIDTH, 0, mmNAV_TARGET_HEIGHT)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
    public static final OpenGLMatrix BACK_NAVMARK_ON_FIELD = OpenGLMatrix
            .translation(mmFIELD_WIDTH, 0, mmNAV_TARGET_HEIGHT)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));

    // VUFORIA KEY (probably should leave this at like the far bottom of the code because it is long)
    public static final String  VUFORIA_KEY = "AUgn7Sz/////AAABmYpi7UFrPkvhi1BYg9enIAuNYi9QI7q7c6kUql4tFKdefbDyh10qAX6SaWeEEMjSEvoPE7IRrSrZSEEeQBdGiXvFLqVfgZcfJCPSvdyIavl0tBMKzw5DXiFNlX4zxGaz8de84jLq9SkDBkbsyELcjttFVmx0RoCfbqKXu72eMLVbKvRdc/aTzK72/amFJHWBGBZ+poyAWHMi7Qn2UHpp6r05D/dG78UhzVAFU1xzXr7p8w4Q2pGnSJbhLNpTd5bSd7ceP4XaZVG2qBwWfuhpwurSK/qEMB7zWjbimtOT+a3t9HffDFNk9oJWwqUVG8UT9ddQv0JMgfNgK6dF+b5YdC5i0qpM9WUOU+sQLDOotlR3";

    public enum GOLD_POSITION {LEFT, CENTER, RIGHT}

    public enum DRIVE_DIRECTION {STRAFE_LEFT, STRAFE_RIGHT, FORWARD, BACKWARDS}
}

package org.firstinspires.ftc.teamcode.RogueOpModes.Autonomous.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RogueLibrary.autonomous.AutonomousUtils;
import org.firstinspires.ftc.teamcode.RogueLibrary.vision.managers.VuforiaManager;
@Disabled

@Autonomous(name = "shittyassrobotvuforiatest", group = "Testing")
public class shittyassrobotvuforiatest extends LinearOpMode {

    private VuforiaManager vuforiaManager;

    private DcMotor FR,
                    FL,
                    BR,
                    BL;

    private static final Orientation targetRotation = Orientation.getOrientation(AutonomousUtils.FRONT_NAVMARK_ON_FIELD, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

    public void runOpMode() {
        vuforiaManager = new VuforiaManager();

        vuforiaManager.initVuforia();
        vuforiaManager.initTrackables();

        FR = hardwareMap.dcMotor.get("1");
        FL = hardwareMap.dcMotor.get("2");
        BR = hardwareMap.dcMotor.get("3");
        BL = hardwareMap.dcMotor.get("4");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        vuforiaManager.startTracking();
        while (opModeIsActive()) {
            VuforiaManager.RobotLocation lastLocation = null;
            lastLocation = vuforiaManager.getRobotLocation();

            if (lastLocation.visibleTarget == "Front-Craters") {
                if (lastLocation.translation.get(1) != -(AutonomousUtils.mmFIELD_WIDTH)) {
                    if (lastLocation.rotation.thirdAngle > -targetRotation.thirdAngle) {
                        FL.setPower(1.0);
                        FR.setPower(-1.0);
                        BL.setPower(1.0);
                        BR.setPower(-1.0);
                    } else if (lastLocation.rotation.thirdAngle < -targetRotation.thirdAngle) {
                        FL.setPower(-1.0);
                        FR.setPower(1.0);
                        BL.setPower(-1.0);
                        BR.setPower(1.0);
                    } else {
                        FL.setPower(0);
                        FR.setPower(0);
                        BL.setPower(0);
                        BR.setPower(0);
                    }
                }else {
                    FL.setPower(0);
                    FR.setPower(0);
                    BL.setPower(0);
                    BR.setPower(0);
                }
            }else {
                FL.setPower(0);
                FR.setPower(0);
                BL.setPower(0);
                BR.setPower(0);
            }
        }
    }

}

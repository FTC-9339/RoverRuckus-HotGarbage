package org.firstinspires.ftc.teamcode.RogueOpModes.Manual.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RogueLibrary.vision.managers.VuforiaManager;

@Disabled
@TeleOp(name = "Vuforia Testing", group = "Testing")
public class VuforiaTesting extends LinearOpMode {
    private VuforiaManager vuforia;

    public void runOpMode() {
        vuforia = new VuforiaManager();
        vuforia.initVuforia(hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
        vuforia.initTrackables();
        telemetry.addLine("Initiated Vuforia");
        telemetry.update();
        waitForStart();
        vuforia.startTracking();
        telemetry.clearAll();
        while (opModeIsActive()) {
            if (vuforia.getRobotLocation() != null) {
                telemetry.addLine("Current Location: " + vuforia.getRobotLocation().toString());
                telemetry.addLine("VuMark Detected: " + vuforia.getRobotLocation().visibleTarget);
            }
            telemetry.update();
        }
    }

}

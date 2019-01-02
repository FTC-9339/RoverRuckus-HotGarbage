package org.firstinspires.ftc.teamcode.RogueOpModes.Manual.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RogueLibrary.vision.managers.TensorflowManager;

@TeleOp(name = "TFOD/Vuforia Testing", group = "Testing")
public class TFODTesting extends LinearOpMode {

    TensorflowManager tfod;

    public void runOpMode() {
        tfod = new TensorflowManager(this, true);
        tfod.getVuforiaManager().initTrackables();
        waitForStart();
        tfod.startDetection();
        tfod.getVuforiaManager().startTracking();
        while (opModeIsActive()) {
            telemetry.clearAll();
            switch (tfod.detectAllMinerals()) {
                case LEFT:
                    telemetry.addLine("Gold Mineral Position: LEFT");
                    break;
                case RIGHT:
                    telemetry.addLine("Gold Mineral Position: RIGHT");
                    break;
                case CENTER:
                    telemetry.addLine("Gold Mineral Position: CENTER");
                    break;
            }
            telemetry.addLine(tfod.getVuforiaManager().getRobotLocation().toString());
        }
    }

}

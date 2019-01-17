package org.firstinspires.ftc.teamcode.RogueOpModes.Manual.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RogueLibrary.vision.managers.TensorflowManager;

@TeleOp(name = "TFOD Testing", group = "Testing")
public class TFODTesting extends LinearOpMode {

    TensorflowManager tfod;
    TensorflowManager.GOLD_POSITION mineralDetected;

    public void runOpMode() {
        tfod = new TensorflowManager(this, true);
        waitForStart();
        tfod.startDetection();
        while (opModeIsActive()) {
            telemetry.clearAll();
            mineralDetected = tfod.detectAllMinerals();
            if (mineralDetected == null) {
                telemetry.addLine("DON'T WATCH AN ANIME CALLED BOKU");
                continue;
            }
            switch (mineralDetected) {
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
        }
        tfod.shutdown();
    }

}

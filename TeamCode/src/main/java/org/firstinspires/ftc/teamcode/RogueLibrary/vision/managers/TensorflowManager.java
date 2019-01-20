package org.firstinspires.ftc.teamcode.RogueLibrary.vision.managers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.RogueLibrary.autonomous.AutonomousUtils;

import java.util.List;

import static org.firstinspires.ftc.teamcode.RogueLibrary.autonomous.AutonomousUtils.LABEL_SILVER_MINERAL;

public class TensorflowManager {

    private TFObjectDetector tfod;
    private TFObjectDetector.Parameters parameters;

    private VuforiaManager vuforia;

    private boolean isTfodActive = false;

    private int goldMineral_x,
                silverMineral_1_x;

    public void startDetection() {
        if (tfod != null) tfod.activate();
        isTfodActive = true;

        goldMineral_x = silverMineral_1_x = -1;
    }

    public TensorflowManager(OpMode opMode, boolean createview) {
        vuforia = new VuforiaManager();

        if (!vuforia.isVuforiaInitialized()) {
            vuforia.initVuforia();
        }

        if (createview) {
            int tfodMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
            parameters = new TFObjectDetector.Parameters(tfodMonitorViewId);

        } else parameters = new TFObjectDetector.Parameters();

        tfod = ClassFactory.getInstance().createTFObjectDetector(parameters, vuforia.getVuforiaLocalizer());
        tfod.loadModelFromAsset(AutonomousUtils.TFOD_MODEL_ASSET, AutonomousUtils.LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public TensorflowManager(OpMode opMode) {new TensorflowManager(opMode, false);}

    public VuforiaManager getVuforiaManager() {return vuforia;}

    public void shutdown() {
        tfod.shutdown();
    }

    public boolean goldMineralInView() {
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getLabel().equals(AutonomousUtils.LABEL_GOLD_MINERAL)) return true;
                }
            }
        }

        return false;
    }

    public GOLD_POSITION detectAllMinerals() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                if (updatedRecognitions.size() == 2) {
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        recognition.getConfidence();
                        if (recognition.getLabel().equals(AutonomousUtils.LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                        } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                        }
                    }
                    if (goldMineralX != -1 && silverMineral1X != -1) {
                        if (goldMineralX > silverMineral1X) {
                            return GOLD_POSITION.LEFT;
                        } else if (goldMineralX < silverMineral1X) {
                            return GOLD_POSITION.CENTER;
                        } else {
                            return GOLD_POSITION.RIGHT;
                        }
                    }
                }
            }
        }
        return null;
    }

    public enum GOLD_POSITION {
        LEFT,
        CENTER,
        RIGHT
    }

}

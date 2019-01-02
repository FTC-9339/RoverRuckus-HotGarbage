package org.firstinspires.ftc.teamcode.RogueLibrary.vision.managers;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.RogueLibrary.autonomous.AutonomousUtils;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class VuforiaManager {

    private VuforiaLocalizer vuforia;
    private VuforiaLocalizer.Parameters parameters;

    boolean vuforiaInitialized = false;
    boolean trackingInitialized = false;

    private VuforiaTrackables targetsRoverRuckus;
    private List<VuforiaTrackable> allTrackables;

    private boolean targetVisible;
    private OpenGLMatrix lastLocation;

    public VuforiaManager() {}

    public void initVuforia() {
        parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = AutonomousUtils.VUFORIA_KEY;
        parameters.cameraDirection = AutonomousUtils.CAMERA_CHOICE;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        vuforiaInitialized = true;
    }

    public void initVuforia(int cameraMonitorViewId) {
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = AutonomousUtils.VUFORIA_KEY;
        parameters.cameraDirection = AutonomousUtils.CAMERA_CHOICE;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        vuforiaInitialized = true;
    }

    public void initVuforia(WebcamName webCamName) {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = AutonomousUtils.VUFORIA_KEY;
        parameters.cameraName = webCamName;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    public VuforiaLocalizer getVuforiaLocalizer() {if (vuforiaInitialized) return vuforia; else return null;}

    public boolean isVuforiaInitialized() {return vuforiaInitialized;}

    public void initTrackables() {
        targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsRoverRuckus);

        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(AutonomousUtils.PHONE_LOCATION_ON_ROBOT, parameters.cameraDirection);
        }

        trackingInitialized = true;
    }

    public boolean isTrackingInitialized() {return trackingInitialized;}

    public void startTracking() {
        targetsRoverRuckus.activate();

        targetVisible = false;
    }

    public RobotLocation getRobotLocation() {
        String target = null;

        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                targetVisible = true;
                target = trackable.getName();
                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF trans = lastLocation.getTranslation();

            // express the rotation of the robot in degrees.
            Orientation rot = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);

            return new RobotLocation(trans, rot, target);
        }
        else {
            return null;
        }

    }

    public static class RobotLocation {
        public static VectorF translation;
        public static Orientation rotation;
        public static String visibleTarget;

        public RobotLocation(VectorF trans, Orientation rot, String visibleTarget) {
            translation = trans;
            rotation = rot;
            this.visibleTarget = visibleTarget;
        }

        public String toString() {
            return "Translation: " + translation.toString() + "\nRotation: " + rotation.toString() + "\nVisible Targets" + visibleTarget;
        }
    }

}

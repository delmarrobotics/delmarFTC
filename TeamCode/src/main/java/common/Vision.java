package common;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
* This file contains support for TensorFlow object recognition and AprilTag recognition
*/
public class Vision {
    private static final String TFOD_MODEL_FILE = "TeamElement1.tflite";
    private static final String[] LABELS = { "Team Element" };

    // AprilTag IDs
    public static final int BLUE_LEFT_TAG   = 1;
    public static final int BLUE_CENTER_TAG = 2;
    public static final int BLUE_RIGHT_TAG  = 3;

    public static final int RED_LEFT_TAG    = 4;   // ToDo check ids
    public static final int RED_CENTER_TAG  = 5;
    public static final int RED_RIGHT_TAG   = 6;

    private TfodProcessor tfod;             // TensorFlow Object Detection processor
    private AprilTagProcessor aprilTag;     // AprilTag Detection processor
    private VisionPortal visionPortal;      // Instance of the vision portal.

    private int gain = 16;                 // camera gain
    private int exposure = 16;             // camera exposure

    Recognition element = null;            // recognized team element

    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    private final ElapsedTime searchTime = new ElapsedTime();

    public LinearOpMode opMode;

    // Constructor
    public Vision (LinearOpMode opMode) {
        this.opMode = opMode;
        initTfod();
    }

    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                .setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        aprilTag = new AprilTagProcessor.Builder()
                .build();


        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(opMode.hardwareMap.get(WebcamName.class, Config.CAMERA));

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessors(tfod, aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.50f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    public void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        opMode.telemetry.addData("# Objects Detected", currentRecognitions.size());
        if (currentRecognitions.size() == 0) {
            opMode.telemetry.addData("Status", "No objects detected");
        }

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            opMode.telemetry.addData(""," ");
            opMode.telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            opMode.telemetry.addData("- Position", "%.0f / %.0f", x, y);
            opMode.telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            opMode.telemetry.addData("- Angle ", "%f  %f",
                    recognition.estimateAngleToObject(AngleUnit.DEGREES),
                    recognition.estimateAngleToObject(AngleUnit.RADIANS));
        }
    }   // end method telemetryTfod()

    public void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        opMode.telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                opMode.telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                opMode.telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                opMode.telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                opMode.telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                opMode.telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                opMode.telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

    }   // end method telemetryAprilTag()

    public void enableTensorFlow() {
        visionPortal.setProcessorEnabled(aprilTag, false);
        visionPortal.setProcessorEnabled(tfod, true);
    }

    public void enableAprilTag() {
        visionPortal.setProcessorEnabled(tfod, false);
        visionPortal.setProcessorEnabled(aprilTag, true);
    }

    public void enableBothVision() {
        visionPortal.setProcessorEnabled(tfod, true);
        visionPortal.setProcessorEnabled(aprilTag, true);
        opMode.telemetry.addData("activeVisionPortal", "tfod %b  aprilTag %b",
                visionPortal.getProcessorEnabled(tfod), visionPortal.getProcessorEnabled(aprilTag));
    }

    public boolean findAprilTag (int tagID) {

        boolean targetFound = false;
        desiredTag  = null;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((tagID < 0) || (detection.id == tagID)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                }
            }
        }

        if (targetFound) {
            //Logger.message("x %f y %f", desiredTag.ftcPose.x, desiredTag.ftcPose.y);
            opMode.telemetry.addLine(String.format("\n==== (ID %d) %s", desiredTag.id, desiredTag.metadata.name));
            opMode.telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", desiredTag.ftcPose.x, desiredTag.ftcPose.y, desiredTag.ftcPose.z));
            opMode.telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", desiredTag.ftcPose.pitch, desiredTag.ftcPose.roll, desiredTag.ftcPose.yaw));
            opMode.telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", desiredTag.ftcPose.range, desiredTag.ftcPose.bearing, desiredTag.ftcPose.elevation));
        } else {
            opMode.telemetry.addLine(String.format("Tag not found"));
        }

        return targetFound;
    }

    public int aprilTagID() { return desiredTag.id; }

    public double aprilTagX() {
        return desiredTag.ftcPose.x;
    }

    public double aprilTagY(){
        return desiredTag.ftcPose.y;
    }

    public double aprilTagYaw() { return desiredTag.ftcPose.yaw; }

    /**
     * Find the object with the highest confidence.
     *
     * @return true if an object was detected
     */
    public boolean findTeamElement (double timeout) {

        element = null;

        List<Recognition> currentRecognitions;
        searchTime.reset();
        while (true) {
            currentRecognitions = tfod.getRecognitions();
            if (currentRecognitions.size() != 0)
                break;
            Logger.message("no team element found");
            if (timeout == 0 || searchTime.milliseconds() >= timeout)
                break;
            opMode.sleep(100);
        }

        currentRecognitions = tfod.getRecognitions();
        if (currentRecognitions.size() == 0)
            return false;

        for (Recognition recognition : currentRecognitions) {
            if (element == null) {
                element = recognition;
            } else {
                if (recognition.getConfidence() > element.getConfidence())
                    element = recognition;
            }
        }
        return true;
    }

    public double findTeamElementAngle() {
        if (element != null) {
            return element.estimateAngleToObject(AngleUnit.DEGREES);
        }
        return 0;
    }

    /**
     * Return the accuracy confidence of the recognized object.
     */
    public float getElementConfidence(){
        if (element != null)
            return element.getConfidence();
        return 0;
    }

    public boolean cameraReady() {
        VisionPortal.CameraState state;
        state = visionPortal.getCameraState();
        Logger.message("camera state %s", state);
        return (state == VisionPortal.CameraState.STREAMING);
    }

    public void calibrateCamera() {

        float confidence = 0;
        int bestExposure = 0;
        int bestGain = 0;

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            opMode.sleep(50);
        }

        for (exposure = 0; exposure < 30 ; exposure += 4) {
            exposureControl.setExposure(16, TimeUnit.MILLISECONDS);
            opMode.sleep(50);

            for (gain = 0; gain < 300; gain += 5 ){
                gainControl.setGain(gain);
                opMode.sleep(500);

                if (findTeamElement(100)) {
                   Logger.message("found - exposure: %d gain: %d  Confidence: %.2f", exposure, gain, element.getConfidence());
                   if (element.getConfidence() < confidence) {
                       bestExposure = exposure;
                       bestGain = gain;
                   }
                } else {
                    Logger.message("not found - exposure: %d gain: %d", exposure, gain);
                 }
            }
        }
        Logger.message("Best setting -  exposure: %d gain: %d", bestExposure, bestGain);
    }
}

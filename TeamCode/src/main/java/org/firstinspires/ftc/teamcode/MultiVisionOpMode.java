
package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.processors.OpenCVProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name="MultiVisionOpMode")
public class MultiVisionOpMode extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    // declare opencv vision portal
//    private VisionPortal OpenCvVisionPortal;
    // declare apriltag vision portal
    private  VisionPortal AprilTagVisionPortal;
    // declare opencv vision processor
//    private OpenCVProcessor OpenCvVisionProcessor;
    // declare apriltag vision processor
    private AprilTagProcessor AprilTagVisionProcessor;


    @Override
    public void runOpMode() {

        // initAuto apriltag vision processor
        AprilTagVisionProcessor = new AprilTagProcessor.Builder()
                .build();
        // initAuto opencv vision processor
//        OpenCvVisionProcessor = new OpenCVProcessor(telemetry);

        // initAuto opencv vision portal
//        OpenCvVisionPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .addProcessor(OpenCvVisionProcessor)
//                .build();



        // enable opencv
//        OpenCvVisionPortal.setProcessorEnabled(OpenCvVisionProcessor, true);


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // get the prop location
//        String location = OpenCvVisionProcessor.getLocation();
//        telemetry.speak(location);
//        telemetry.update();

        // disable opencv
//        OpenCvVisionPortal.setProcessorEnabled(OpenCvVisionProcessor, false);
//        OpenCvVisionPortal.close();

        // initAuto apriltag vision portal
        AprilTagVisionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .enableLiveView(true)
//                .setCameraResolution(new Size(1280,720))
                .setCameraResolution(new Size(1920,1080))
                .addProcessor(AprilTagVisionProcessor)
                .build();

        // enable apriltag
        AprilTagVisionPortal.setProcessorEnabled(AprilTagVisionProcessor, true);





        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (isStopRequested()) {
                telemetry.addData("STOP", "STOP");
                telemetry.update();
                AprilTagVisionPortal.setProcessorEnabled(AprilTagVisionProcessor, false);
                AprilTagVisionPortal.close();

            }

            // check apriltag detections
            telemetryAprilTag();

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("Location", location);
            telemetry.update();



        }
    }

    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = AprilTagVisionProcessor.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 5) {
                double[] result = corrected_x_y(detection);
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                telemetry.addLine(String.format("CX CY %6.1f %6.1f", result[0], result[1]));
                telemetry.addData("X", 60.25 - result[1]);
                telemetry.addData("Y", -35.41 + result[0]);
            } else {
             //   telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
             //   telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
            double[] result = corrected_x_y(detection);

        }   // end for() loop

    }   // end method telemetryAprilTag()

    private double[] corrected_x_y(AprilTagDetection det) {
        // returns an array of length 2 (x' and y')
        double x_prime = 0;
        double y_prime = 0;
        double y_offset = 8.5;

        // calculation stuff
        double corrected_range = Math.sqrt(Math.pow(det.ftcPose.y, 2) + Math.pow(det.ftcPose.x, 2));
        double theta_rad = Math.toRadians(det.ftcPose.yaw - det.ftcPose.bearing);
        x_prime = Math.sin(theta_rad) * corrected_range - y_offset;
        y_prime = Math.cos(theta_rad) * corrected_range;


        double[] result = new double[2];
        result[0] = x_prime;
        result[1] = y_prime;
        return result;
    };

}

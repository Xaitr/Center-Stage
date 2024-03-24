package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

import java.util.ArrayList;

@Autonomous
public class AsyncBlueLeft extends LinearOpMode {

    //This enum defines steps of the trajectories
    enum State {
        BACKBOARD_DROP, //First drop of the yellow preload on the backboard
        PREDROP, //Then place the purple preload on the spikemark
        GENERALSTACK, //Then drive to in front of the white stacks
        INTAKESTACK, //Drive into stack with intake on
        BACKBOARD_STACK, //Drive to backboard to place white pixels
        PARK, //Park in backstage
        IDLE //Robot enters idle when finished
    }

    //Instantiate our driveState
    State driveState = State.IDLE;

    //Declare purple preload servo
    private Servo preDropLeft = null;

    //Instantiate our lift class
    PidControl2 lift =new PidControl2();

    //Timer for in between trajectories
    ElapsedTime driveTimer = new ElapsedTime();

    //Declare our April tag & OpenCV vision portal
    private VisionPortal OpenCvVisionPortal;

    //Declare both webcams
    private WebcamName webcam1, webcam2;

    //Declare our AprilTag Processor
    private AprilTagProcessor aprilProcessor;

    //Declare our colour processor
    private BlueProcessor blueProcessor;

    @Override
    public void runOpMode() throws InterruptedException {
        //Assign both webcams
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");

        //init AprilTag & Colour processor
        aprilProcessor = new AprilTagProcessor.Builder()
                .build();
        blueProcessor = new BlueProcessor();

        //init VisionPortal
        OpenCvVisionPortal = new VisionPortal.Builder()
                .setCamera(webcam1)
                .setCamera(webcam2)
                .addProcessor(blueProcessor)
                .addProcessor(aprilProcessor)
                .build();

        //Enable colour processor and webcam1
        OpenCvVisionPortal.setProcessorEnabled(blueProcessor, true);
        OpenCvVisionPortal.setActiveCamera(webcam1);

        waitForStart();

        //Turn off Vision Portal to conserve resources
        OpenCvVisionPortal.stopStreaming();

        //Need to test if this prevents crashing when stopping autonomous
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

        }
    }
}

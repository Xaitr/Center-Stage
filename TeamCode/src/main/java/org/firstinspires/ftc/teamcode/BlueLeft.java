package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class BlueLeft extends LinearOpMode {
    OpenCvCamera camera;

    @Override
    public void runOpMode() throws InterruptedException {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        OpenCvblue detector = new OpenCvblue (telemetry);
        camera.setPipeline(detector);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener();
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(31.1, 61.6, Math.toRadians(90));


        {
            @Override

            public void onOpened()
            {
                camera.startStreaming(1920,1080, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });


        waitForStart();


        switch (detector.getLocation()) {
            case LEFT:
                telemetry.addData("Left side","proceed");
                telemetry.update();




                break;
            case RIGHT:
                telemetry.addData("Right Side","proceed");
                telemetry.update();
                break;



            case MIDDLE:
                telemetry.addData("Middle","proceed");
                telemetry.update();
                break;



            case NOT_FOUND:
                telemetry.addData("not found","proceed");
                telemetry.update();
        }
        camera.stopStreaming();
    }
}
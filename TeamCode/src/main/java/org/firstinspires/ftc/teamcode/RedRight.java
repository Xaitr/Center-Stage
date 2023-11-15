package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class RedRight extends LinearOpMode {
    Outake outake = new Outake();
    Intake intake = new Intake();
    PidControl2 Lift = new PidControl2();
    OpenCvCamera camera;

    @Override
    public void runOpMode() throws InterruptedException {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        OpenCvblue detector = new OpenCvblue(telemetry);
        camera.setPipeline(detector);
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(13, -65, Math.toRadians(180));
        robot.setPoseEstimate(startPose);
        Pose2d prePark = new Pose2d(0, 0, 0);
        intake.init(hardwareMap);


        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
        /*
        TrajectorySequence Right = robot.trajectorySequenceBuilder(startPose)
            .lineToConstantHeading(new Vector2d(14, -34))
            .splineToConstantHeading(new Vector2d(34, -34), Math.toRadians(0))
            .addTemporalMarker(() -> {
                    intake.sReject();
                    sleep(1000); // stops program for 1000 milliseconds
                    intake.RejectOff();
                })
                .waitSeconds(1)
                // spit out pixel
            .splineTo(new Vector2d(48, -34), Math.toRadians(0))

            // put pixel on board here
            .strafeLeft(25)


            .build();
 */
        TrajectorySequence Right = robot.trajectorySequenceBuilder(startPose)
                .back(6)
                .splineToConstantHeading(new Vector2d(37, -34), Math.toRadians(0))
                // spit out pixel here
                .addTemporalMarker(() -> {
                    intake.Reject();
                })
                .waitSeconds(0.05)
                .addTemporalMarker(() -> {
                    intake.RejectOff();
                })
                .strafeTo(new Vector2d(48, -37))
                // place pixel on board
                .build();
        TrajectorySequence Left = robot.trajectorySequenceBuilder(startPose)
                .back(0.5)
                .splineToConstantHeading(new Vector2d(16, -32), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    intake.Reject();
                })
                .waitSeconds(0.05)
                .addTemporalMarker(() -> {
                    intake.Reject();
                })
                // spit out pixel here
                .strafeTo(new Vector2d(48, -40))
                // put pixel on board

                .build();
        TrajectorySequence Park = robot.trajectorySequenceBuilder(new Pose2d(48, -54, Math.toRadians(180)))
                .strafeLeft(13)
                .build();
        TrajectorySequence Middle = robot.trajectorySequenceBuilder(startPose)
                .back(5)
                .splineToConstantHeading(new Vector2d(26, -25.5), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    intake.Reject();
                })
                .waitSeconds(0.05)
                .addTemporalMarker(() -> {
                    intake.RejectOff();
                })
                .lineToConstantHeading(new Vector2d(47, -34))
        // put pixel on board
                 .build();
        waitForStart();


        switch (detector.getLocation()) {
            case LEFT:
                telemetry.addData("Left side", "proceed"); // open cv detects left spike
                telemetry.update();
                robot.followTrajectorySequence(Left);
                robot.followTrajectorySequence(Park);
                break;


            case RIGHT:
                telemetry.addData("Right Side", "proceed");
                telemetry.update();
                robot.followTrajectorySequence(Right);
                robot.followTrajectorySequence(Park);
                break;


            case MIDDLE:
                telemetry.addData("Middle", "proceed");
                telemetry.update();
                robot.followTrajectorySequence(Middle);
                robot.followTrajectorySequence(Park);
                break;




            case NOT_FOUND:
                telemetry.addData("not found", "proceed");
                telemetry.update();
                robot.followTrajectorySequence(Right);
        }
        camera.stopStreaming();
    }
}
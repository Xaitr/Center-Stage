import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Driving;
import org.firstinspires.ftc.teamcode.OpenCvblue;
import org.firstinspires.ftc.teamcode.Outake;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class RedLeft extends LinearOpMode {
    Outake outake = new Outake();
    Driving driving = new Driving();
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
        Pose2d startPose = new Pose2d(-34, -61, Math.toRadians(270));


        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
        TrajectorySequence Left = robot.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-36, -33), Math.toRadians(0))
                .turn(-1.6)
                .addTemporalMarker(() -> {
                    outake.setPower(-1);
                    sleep(1000); // stops program for 1000 miliseconds
                    outake.setPower(0);
                })
                .waitSeconds(1)
                // spit out pixel here
                .strafeTo(new Vector2d(-34,-10))
                .lineTo(new Vector2d(40, -15))
                .splineToConstantHeading(new Vector2d(60, -10), Math.toRadians(0))
                .build();

        TrajectorySequence Right = robot.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-34, -33), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    outake.setPower(-1);
                    sleep(1000); // stops program for 1000 miliseconds
                    outake.setPower(0);
                })
                .waitSeconds(1)
                // spit out pixel here
                .strafeTo(new Vector2d(-34,-10))
                .lineTo(new Vector2d(40, -15))
                .splineToConstantHeading(new Vector2d(60, -10), Math.toRadians(0))
                .build();

        TrajectorySequence Middle = robot.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-34, -33), Math.toRadians(90))
                .addTemporalMarker(() -> {
                    outake.setPower(-1);
                    sleep(1000); // stops program for 1000 miliseconds
                    outake.setPower(0);
                })
                .waitSeconds(1)
                // spit out pixel here
                .strafeTo(new Vector2d(-34,-10))
                .lineTo(new Vector2d(40, -15))
                .splineToConstantHeading(new Vector2d(60, -10), Math.toRadians(0))
                .build();

        waitForStart();


        switch (detector.getLocation()) {
            case LEFT:
                telemetry.addData("Left side", "proceed"); // open cv detects left spike
                telemetry.update();
                robot.followTrajectorySequence(Left);
                break;


            case RIGHT:
                telemetry.addData("Right Side", "proceed");
                telemetry.update();
                break;
            //robot.followTrajectorySequence(Right);


            case MIDDLE:
                telemetry.addData("Middle", "proceed");
                telemetry.update();
                break;
            //robot.followTrajectorySequence(Middle);


            case NOT_FOUND:
                telemetry.addData("not found", "proceed");
                telemetry.update();
                robot.followTrajectorySequence(Left);
        }
        camera.stopStreaming();
    }
}

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.OpenCvblue.Location.MIDDLE;
import static org.firstinspires.ftc.teamcode.OpenCvblue.Location.RIGHT;

import android.graphics.Canvas;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;
import org.openftc.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvSwitchableWebcam;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class CameraTest extends LinearOpMode {


    private WebcamName webcam1, webcam2;
    AprilTagPipeline pipeline = new AprilTagPipeline();
    // private OpenCvCamera.AsyncCameraOpenListener
    private AprilTagProcessor aprilTag;
    Outake outake = new Outake();
    Intake intake = new Intake();
    PidControl2 lift = new PidControl2();
    OpenCvCamera camera;


    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int one = 1;
    int two = 2;
    int three = 3;

    AprilTagDetection tagOfInterest = null;
    private VisionPortal visionPortal;

    // Enum to represent lift state
    private enum LiftState {
        LIFT_EXTEND,
        BOX_EXTEND,
        LIFT_DUMP,
        BOX_RETRACT,
        LIFT_RETRACT,
        LIFT_RETRACTED,
        LIFT_DONE
    }

    //Timer for waiting for pixels to spin out
    ElapsedTime liftTimer = new ElapsedTime();
    LiftState liftState = LiftState.LIFT_EXTEND;
    private int liftHeight = 0;
    private CRServo IOservo = null;
    private DcMotor leftLift = null;
    private Servo rightServo = null;

 //   public void doCameraSwitching() {

//        visionPortal = new VisionPortal.Builder()
//                .setCamera(webcam2)
//                .addProcessor(aprilTag)
//                .build();
//
//    }


    @Override
    public void runOpMode() throws InterruptedException {
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
       OpenCvSwitchableWebcam camera = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId, webcam1, webcam2);
        OpenCvblue detector = new OpenCvblue(telemetry);
       //  OpenCv detector2 = new OpenCv(telemetry);
        camera.setPipeline(detector);
        pipeline.telemetry = telemetry;


        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(13, 65, Math.toRadians(180));
        robot.setPoseEstimate(startPose);
        Pose2d prePark = new Pose2d(0, 0, 0);
        intake.init(hardwareMap);
        leftLift = hardwareMap.get(DcMotorEx.class, "left_lift");
        IOservo = hardwareMap.get(CRServo.class, "IOservo");
        rightServo = hardwareMap.get(Servo.class, "Right_outtake");
        lift.init(hardwareMap);


        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });


        waitForStart();
        camera.setActiveCamera(webcam1);
      //  List<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> currentDetections = aprilTag.getDetections();

        switch (detector.getLocation()) {
            case NOT_FOUND:
                sleep(10000);
                camera.setActiveCamera(webcam2);

                // camera.setPipeline(detector2);
                pipeline.initAprilTag2(webcam2);


             //   sleep(2000);
              //  sleep(10000);
                break;

            case RIGHT:
                sleep(10000);
                camera.setActiveCamera(webcam2);

                // camera.setPipeline(detector2);
                pipeline.initAprilTag2(webcam2);


               // sleep(2000);
            //    sleep(10000);
                break;

            case LEFT:
                sleep(10000);
                camera.setActiveCamera(webcam2);

               // camera.setPipeline(detector2);
                pipeline.initAprilTag2(webcam2);


               // sleep(2000);
               // sleep(10000);
                break;

            case MIDDLE:
                sleep(10000);
                camera.setActiveCamera(webcam2);

                //camera.setPipeline(detector2);
                pipeline.initAprilTag2(webcam2);


              //  sleep(2000);
              //  sleep(10000);
                break;
        }
        // TODO (Jacob / Ben): Turn off pipeline???
        camera.setPipeline(null);
        while (opModeIsActive()) {

           pipeline.telemetryAprilTag();

            // Push telemetry to the Driver Station.
            telemetry.update();

            // Save CPU resources; can resume streaming when needed.
            if (gamepad1.dpad_down) {
                visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
                visionPortal.resumeStreaming();
            }

            // Share the CPU.
            sleep(20);
        }
    }
}

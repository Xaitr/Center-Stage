package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class TestingAsync extends OpMode {

    private WebcamName webcam1;
    Outake outake = new Outake();
    Intake intake = new Intake();

    private Servo preDropLeft = null;
    PidControl2 lift =new PidControl2();
    OpenCvCamera camera;

    // AprilTag aprilTag = new AprilTag();
    // private VisionPortal visionPortal;

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

    // AprilTagDetection tagOfInterest = null;


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
    // public void doCameraSwitching() {
    //  if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
    //    visionPortal.setActiveCamera(webcam2);
    //   }}

    SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
    @Override
    public void init() {
        WebcamName webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        //      webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        //  OpenCvSwitchableWebcam camera = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId, webcam1, webcam2);
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcam1);
        OpenCvblue detector = new OpenCvblue(telemetry);
        camera.setPipeline(detector);
        //  aprilTag.initAprilTag();
        Pose2d startPose = new Pose2d(13, 65, Math.toRadians(180));
        robot.setPoseEstimate(startPose);
        Pose2d prePark = new Pose2d(0, 0, 0);
        intake.init(hardwareMap);
        leftLift = hardwareMap.get(DcMotorEx.class, "left_lift");
        IOservo = hardwareMap.get(CRServo.class, "IOservo");
        rightServo = hardwareMap.get(Servo.class, "Right_outtake");
        preDropLeft = hardwareMap.get(Servo.class, "preDropLeft");
        lift.initAuto(hardwareMap);




        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
        /*
        TrajectorySequence Left = robot.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(14, 34))
                .splineToConstantHeading(new Vector2d(34, 34), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    intake.Reject();
                    sleep(1000); // stops program for 1000 milliseconds
                    intake.RejectOff();
                })
                .waitSeconds(1)
                //  Spit out pixel
                .splineTo(new Vector2d(48, 34), Math.toRadians(0))

                // put pixel on board here
                .strafeRight(25)


                .build();
*/
        Trajectory BackBoardDrop1 = robot.trajectoryBuilder(startPose)
                .strafeTo(new Vector2d(52, 40))
                .build();
        //put pixel on backboard
        TrajectorySequence PreDropLeft = robot.trajectorySequenceBuilder(BackBoardDrop1.end())
                .lineTo(new Vector2d(33, 32))
                .build();
        //put pixel on left line

        TrajectorySequence Generalposition = robot.trajectorySequenceBuilder(PreDropLeft.end())
                .lineTo(new Vector2d(33, 8))
                .build();
        //move to general position before white pixels from left line

        TrajectorySequence WhiteStackPath = robot.trajectorySequenceBuilder(Generalposition.end())
                .lineTo(new Vector2d(-56, 8))
                .build();
        //intake pixel off white stack

        TrajectorySequence StacktoBackBoard = robot.trajectorySequenceBuilder(WhiteStackPath.end())
                .strafeTo(new Vector2d(38, 8))
                .splineToConstantHeading(new Vector2d(50, 32), Math.toRadians(0))
                .build();
        // put white pixels on backboardboard
        TrajectorySequence Park = robot.trajectorySequenceBuilder(StacktoBackBoard.end())
                .splineToConstantHeading(new Vector2d(56, 60), Math.toRadians(0))
                .build();
        //park


        TrajectorySequence Right = robot.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(50, 32))
                //place pixel on backboard
                .lineTo(new Vector2d(10, 32))
                .turn(0)
                //place pixel on line

                // .lineTo(new Vector2d(33,8))
                //.lineTo(new Vector2d (-56, 8))
                //grab pixel of stack
                //.strafeTo(new Vector2d(16,8))
                //.strafeTo(new Vector2d(38, 8))
                //.splineToConstantHeading(new Vector2d(50,32), Math.toRadians(0))
                //.turn(0)
                //place pixel on backboard
                //.splineToConstantHeading(new Vector2d(56,60), Math.toRadians(0))

                .build();
        //TrajectorySequence Park = robot.trajectorySequenceBuilder(new Pose2d(51, 37, Math.toRadians(180)))
        //.lineToConstantHeading(new Vector2d(48,37))
        //  .splineToConstantHeading(new Vector2d(56,63), Math.toRadians(0))
        // .build();
        TrajectorySequence Middle = robot.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(50, 32))
                //place pixel on backboard
                .lineTo(new Vector2d(15, 22))
                .turn(0)
                //place pixel on line

                //  .lineTo(new Vector2d(33, 8))
                // .lineTo(new Vector2d (-56, 8))
                //grab pixels off stack
                //.strafeTo(new Vector2d(16,8))
                //.strafeTo(new Vector2d(38, 8))
                //.splineToConstantHeading(new Vector2d(50,32), Math.toRadians(0))
                //place pixel on backboard
                //.splineToConstantHeading(new Vector2d(56,60), Math.toRadians(0))
                //park
                .build();

        robot.followTrajectoryAsync(BackBoardDrop1);



    }
        public void loop()  { robot.update();
    }

}
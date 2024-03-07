package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.OpenCvblue.Location.MIDDLE;
import static org.firstinspires.ftc.teamcode.OpenCvblue.Location.RIGHT;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.vision.VisionPortal;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvSwitchableWebcam;

@Autonomous
public class BlueLeft extends LinearOpMode {

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


    @Override
    public void runOpMode() throws InterruptedException {
        WebcamName webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
  //      webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
      //  OpenCvSwitchableWebcam camera = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId, webcam1, webcam2);
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcam1);
        OpenCvblue detector = new OpenCvblue (telemetry);
        camera.setPipeline(detector);
      //  aprilTag.initAprilTag();
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(13, 65, Math.toRadians(180));
        robot.setPoseEstimate(startPose);
        Pose2d prePark = new Pose2d(0,0,0);
        intake.init(hardwareMap);
        leftLift = hardwareMap.get(DcMotorEx.class, "left_lift");
        IOservo = hardwareMap.get(CRServo.class, "IOservo");
        rightServo = hardwareMap.get(Servo.class, "Right_outtake");
       preDropLeft = hardwareMap.get(Servo.class,  "preDropLeft");
        lift.init(hardwareMap);




        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1920,1080, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        //TrajectorySequence Left = robot.trajectorySequenceBuilder(startPose)

       TrajectorySequence BackBoardDropLeft = robot.trajectorySequenceBuilder(startPose)
               .strafeTo(new Vector2d(52,42))
               .build();
      //put pixel on backboard
       TrajectorySequence PreDropLeft = robot.trajectorySequenceBuilder(BackBoardDropLeft.end())
                .lineTo(new Vector2d(33, 32))
                .build();
       //put pixel on left line

        TrajectorySequence GeneralpositionLeft = robot.trajectorySequenceBuilder(PreDropLeft.end())
               .lineTo(new Vector2d(33, 8))
                .build();
        //move to general position before white pixels from left line

        TrajectorySequence WhiteStackPathLeft = robot.trajectorySequenceBuilder(GeneralpositionLeft.end())
              .lineTo(new Vector2d (-56, 8))
                .build();
                //intake pixel off white stack

                TrajectorySequence StacktoBackBoardLeft = robot.trajectorySequenceBuilder(WhiteStackPathLeft.end())
                .strafeTo(new Vector2d(38, 8))
                .splineToConstantHeading(new Vector2d(50,32), Math.toRadians(0))
                        .build();
                // put white pixels on backboardboard
                TrajectorySequence ParkLeft = robot.trajectorySequenceBuilder(StacktoBackBoardLeft.end())
               .splineToConstantHeading(new Vector2d(56,60), Math.toRadians(0))
                        .build();
                //park


        //TrajectorySequence Right = robot.trajectorySequenceBuilder(startPose)
        TrajectorySequence BackBoardDropRight = robot.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(52,30))
                //place pixel on backboard
                .build();
        TrajectorySequence PreDropRight = robot.trajectorySequenceBuilder(BackBoardDropRight.end())
                .lineTo(new Vector2d(10, 35))
                .build();
                //place pixel on line
        TrajectorySequence GeneralpositionRight = robot.trajectorySequenceBuilder(PreDropRight.end())
                .lineTo(new Vector2d(33, 8))
                .build();
        //move to general position before white pixels from left line
        TrajectorySequence WhiteStackPathRight = robot.trajectorySequenceBuilder(GeneralpositionRight.end())
                .lineTo(new Vector2d (-56, 8))
                .build();
        //intake pixel off white stack
        TrajectorySequence StacktoBackBoardRight = robot.trajectorySequenceBuilder(WhiteStackPathRight.end())
                .strafeTo(new Vector2d(38, 8))
                .splineToConstantHeading(new Vector2d(50,32), Math.toRadians(0))
                .build();
        // put white pixels on backboardboard
        TrajectorySequence ParkRight = robot.trajectorySequenceBuilder(StacktoBackBoardRight.end())
                .splineToConstantHeading(new Vector2d(46,60), Math.toRadians(0))
                .build();
        //park

        //TrajectorySequence Middle = robot.trajectorySequenceBuilder(startPose)
        TrajectorySequence BackBoardDropMid = robot.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(52,36))
                .build();
                //place pixel on backboard
        TrajectorySequence PreDropMid = robot.trajectorySequenceBuilder(BackBoardDropMid.end())
                .lineTo(new Vector2d(20, 32))
                .build();
        //place pixel on line
        TrajectorySequence GeneralpositionMid = robot.trajectorySequenceBuilder(PreDropMid.end())
                .lineTo(new Vector2d(33, 8))
                .build();
        //move to general position before white pixels from left line

        TrajectorySequence WhiteStackPathMid = robot.trajectorySequenceBuilder(GeneralpositionMid.end())
                .lineTo(new Vector2d (-56, 8))
                .build();
        //intake pixel off white stack

        TrajectorySequence StacktoBackBoardMid = robot.trajectorySequenceBuilder(WhiteStackPathMid.end())
                .strafeTo(new Vector2d(38, 8))
                .splineToConstantHeading(new Vector2d(50,32), Math.toRadians(0))
                .build();
        // put white pixels on backboardboard
        TrajectorySequence ParkMid = robot.trajectorySequenceBuilder(StacktoBackBoardMid.end())
                .splineToConstantHeading(new Vector2d(56,60), Math.toRadians(0))
                .build();
        //park

        waitForStart();
        // camera.setActiveCamera(webcam1);




            switch (detector.getLocation()) {
            case LEFT:
                telemetry.addData("Left side","proceed"); // open cv detects left spike
                telemetry.update();
                 robot.followTrajectorySequence(BackBoardDropLeft);
               while(liftState != BlueLeft.LiftState.LIFT_DONE){
                    switch (liftState) {
                        case LIFT_EXTEND:
                            //Extend lift
                            liftHeight = LiftConstants.liftAuto;
                            //Check if lift has fully extended
                            if (Math.abs(leftLift.getCurrentPosition() - liftHeight) < 15) {
                                //Deploy box
                                liftState = BlueLeft.LiftState.BOX_EXTEND;
                                lift.AutoBoxReady();
                            }
                            break;
                        case BOX_EXTEND:
                            //Wait for servo to reach position
                            if (rightServo.getPosition() == LiftConstants.AutoBoxReady) {
                                liftState = BlueLeft.LiftState.LIFT_DUMP;
                            }
                            break;
                        case LIFT_DUMP:
                            liftState = BlueLeft.LiftState.BOX_RETRACT;
                            //Turn on Outtake Servo
                            IOservo.setPower(-1);
                            //Reset outtake timer
                            liftTimer.reset();
                            break;
                        case BOX_RETRACT:
                            //Wait for pixels to spin out
                            if (liftTimer.seconds() >= LiftConstants.dumpTime) {
                                //Turn off Outtake Servo
                                IOservo.setPower(0);
                                lift.retractBox();
                                liftState = LiftState.LIFT_RETRACT;
                                liftTimer.reset();
                            }
                            break;
                        case LIFT_RETRACT:
                            //Retract Box
                            // Wait for servo to return to Idle
                            if (liftTimer.seconds() >= 0.6) {
                                liftState = LiftState.LIFT_RETRACTED;
                                liftHeight = LiftConstants.liftRetracted;
                            }
                            break;
                        case LIFT_RETRACTED:
                            //Retract Lift

                            //Wait for Lift to return to idle
                            if (Math.abs(leftLift.getCurrentPosition() - LiftConstants.liftRetracted) < 10) {
                                liftState = LiftState.LIFT_DONE;
                            }
                            break;
                    }
                    lift.setHeight(liftHeight);
               }
               lift.disableMotors();

                robot.followTrajectorySequence(PreDropLeft);
                preDropLeft.setPosition(0.85);
                sleep(1000);

//                robot.followTrajectorySequence(Generalposition);
//
//                robot.followTrajectorySequence(WhiteStackPath);
//                put in stack muncher
//                robot.followTrajectorySequence(StacktoBackBoard);

               robot.followTrajectorySequence(ParkLeft);
               break;

            case RIGHT:
                telemetry.addData("Right Side","proceed");
                telemetry.update();
                robot.followTrajectorySequence(BackBoardDropRight);
                while(liftState != BlueLeft.LiftState.LIFT_DONE){
                    switch (liftState) {
                        case LIFT_EXTEND:
                            //Extend lift
                            liftHeight = LiftConstants.liftAuto;
                            //Check if lift has fully extended
                            if (Math.abs(leftLift.getCurrentPosition() - liftHeight) < 15) {
                                //Deploy box
                                liftState = BlueLeft.LiftState.BOX_EXTEND;
                                lift.AutoBoxReady();
                            }
                            break;
                        case BOX_EXTEND:
                            //Wait for servo to reach position
                            if (rightServo.getPosition() == LiftConstants.AutoBoxReady) {
                                liftState = BlueLeft.LiftState.LIFT_DUMP;
                            }
                            break;
                        case LIFT_DUMP:
                            liftState = BlueLeft.LiftState.BOX_RETRACT;
                            //Turn on Outtake Servo
                            IOservo.setPower(-1);
                            //Reset outtake timer
                            liftTimer.reset();
                            break;
                        case BOX_RETRACT:
                            //Wait for pixels to spin out
                            if (liftTimer.seconds() >= LiftConstants.dumpTime) {
                                //Turn off Outtake Servo
                                IOservo.setPower(0);
                                lift.retractBox();
                                liftState = BlueLeft.LiftState.LIFT_RETRACT;
                                liftTimer.reset();
                            }
                            break;
                        case LIFT_RETRACT:
                            //Retract Box

                            // Wait for servo to return to Idle
                            if (liftTimer.seconds() >= 0.6) {
                                liftState = BlueLeft.LiftState.LIFT_RETRACTED;
                                liftHeight = LiftConstants.liftRetracted;
                            }
                            break;
                        case LIFT_RETRACTED:
                            //Retract Lift

                            //Wait for Lift to return to idle
                            if (Math.abs(leftLift.getCurrentPosition() - LiftConstants.liftRetracted) < 10) {
                                liftState = BlueLeft.LiftState.LIFT_DONE;
                            }
                            break;
                    }
                    lift.setHeight(liftHeight);
                }
                lift.disableMotors();

                robot.followTrajectorySequence(PreDropRight);
                preDropLeft.setPosition(0.85);
                sleep(1000);

                robot.followTrajectorySequence(ParkRight);
                break;


            case MIDDLE:
                telemetry.addData("Middle","proceed");
                telemetry.update();
                robot.followTrajectorySequence(BackBoardDropMid);
                while(liftState != BlueLeft.LiftState.LIFT_DONE){
                    switch (liftState) {
                        case LIFT_EXTEND:
                            //Extend lift
                            liftHeight = LiftConstants.liftAuto;
                            //Check if lift has fully extended
                            if (Math.abs(leftLift.getCurrentPosition() - liftHeight) < 15) {
                                //Deploy box
                                liftState = BlueLeft.LiftState.BOX_EXTEND;
                                lift.extendBox();
                            }
                            break;
                        case BOX_EXTEND:
                            //Wait for servo to reach position
                            if (rightServo.getPosition() == LiftConstants.BoxReady) {
                                liftState = BlueLeft.LiftState.LIFT_DUMP;
                            }
                            break;
                        case LIFT_DUMP:
                            liftState = BlueLeft.LiftState.BOX_RETRACT;
                            //Turn on Outtake Servo
                            IOservo.setPower(-1);
                            //Reset outtake timer
                            liftTimer.reset();
                            break;
                        case BOX_RETRACT:
                            //Wait for pixels to spin out
                            if (liftTimer.seconds() >= LiftConstants.dumpTime) {
                                //Turn off Outtake Servo
                                IOservo.setPower(0);
                                lift.retractBox();
                                liftState = BlueLeft.LiftState.LIFT_RETRACT;
                                liftTimer.reset();
                            }
                            break;
                        case LIFT_RETRACT:
                            //Retract Box

                            // Wait for servo to return to Idle
                            if (liftTimer.seconds() >= 0.6) {
                                liftState = BlueLeft.LiftState.LIFT_RETRACTED;
                                liftHeight = LiftConstants.liftRetracted;
                            }
                            break;
                        case LIFT_RETRACTED:
                            //Retract Lift

                            //Wait for Lift to return to idle
                            if (Math.abs(leftLift.getCurrentPosition() - LiftConstants.liftRetracted) < 10) {
                                liftState = BlueLeft.LiftState.LIFT_DONE;
                            }
                            break;
                    }
                    lift.setHeight(liftHeight);
                }
                lift.disableMotors();

                robot.followTrajectorySequence(PreDropMid);
                preDropLeft.setPosition(0.85);
                sleep(1000);

               robot.followTrajectorySequence(ParkMid);
                break;



            case NOT_FOUND:
                telemetry.addData("not found","proceed");
                telemetry.update();

        }
        camera.stopStreaming();


        }
}
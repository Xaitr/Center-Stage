package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.LiftConstants.StackMuncherReturn;
import static org.firstinspires.ftc.teamcode.LiftConstants.backPincherClose;
import static org.firstinspires.ftc.teamcode.LiftConstants.backPincherOpen;
import static org.firstinspires.ftc.teamcode.LiftConstants.frontPincherClose;
import static org.firstinspires.ftc.teamcode.LiftConstants.frontPincherOpen;
import static org.firstinspires.ftc.teamcode.LiftConstants.liftRetracted;
import static org.firstinspires.ftc.teamcode.LiftConstants.wristIdle;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous
public class RedLeftAsync extends LinearOpMode {

    //This enum defines steps of the trajectories
    enum State {
        BACKBOARD_DROP, //First drop of the yellow preload on the backboard
        PREDROP, //Then place the purple preload on the spikemark
        GENERAL_STACK, //Then drive to the white stacks
        APRIL_TAG,
        PARK,
        STACK2,
        INTAKE_STACK,
        APRIL_TAG2,
        BACKBOARD_DROP2, //Drive to backstage to place white pixels
        IDLE //Robot enters idle when finished
    }

    private IMU imu = null;
    private DigitalChannel boxBeam = null;
    Pose2d poseEstimate;

    //Instantiate our driveState
    State driveState = State.PREDROP;

    //Declare purple preload servo
    private Servo preDropLeft = null;

    //Declare stack muncher servo
    private Servo DIservo;

    //Instantiate our lift class
    PidControl2 lift = new PidControl2();

    //This enum defines steps of the lift
    private enum LiftState {
        LIFT_IDLE,
        CLOSE_PINCHERS,
        LIFT_EXTEND,
        BOX_EXTEND,
        LIFT_DUMP,
        BOX_RETRACT,
        LIFT_RETRACT,
    }
    //Creating lift variables and servos
    ElapsedTime liftTimer = new ElapsedTime();
    LiftState liftState = LiftState.LIFT_IDLE;
    private int liftHeight, storeLiftHeight = 0;
    private CRServo transfer = null;
    private Servo frontPincher, backPincher, wrist = null;
    private DcMotor leftLift,intake = null;
    private double alternateWristAngle = 0;

    //Timer for in between trajectories
    ElapsedTime driveTimer = new ElapsedTime();

    //Timer for periodically updating the heading with the imu
    ElapsedTime imuTimer = new ElapsedTime();
    private double headingInterval = 5;
    private double x = 0;
    private double Q = 0.1;
    private double R = 0.4;
    private double p = 0.2;
    private double K = 0.4;

    private double xPrevious = x;
    private double pPrevious = p;
    private double u = 0;
    private double z = 0;


    //Lets the lift state machine know when the trajectory to the backboard is finished
    private boolean readyToDrop = false;

    private boolean ExtraCycle = false;

    //Time for robot to drive away and clear the backboard
    private double dropTime = 1.5;

    //Declare our April tag & OpenCV vision portal
    private VisionPortal OpenCvVisionPortal;

    private VisionPortal AprilTagVisionPortal;

    //Declare both webcams
    private WebcamName webcam1, webcam2;

    //Declare our AprilTag Processor
    private AprilTagProcessor aprilProcessor;
    double x_prime = 0;
    double y_prime = 0;
    double y_offset = 8.5;
    private Vector2d[] aprilTagLibrary = {new Vector2d(60.25,-29.41), new Vector2d(60.25, -35.41), new Vector2d(60.25, -41.41)};
    private RevBlinkinLedDriver Blinky = null;


    //Declare our colour processor
    private RedProcessorFrontStage redProcessorFrontStage;

    @Override
    public void runOpMode() throws InterruptedException {
        //Instantiate SampleMecanumDrive
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        //Define robots starting position and orientation
        Pose2d startPose = new Pose2d(-40, -62, Math.toRadians(270));
        robot.setPoseEstimate(startPose);



        TrajectorySequence PreDropLeft = robot.trajectorySequenceBuilder(startPose)
                .back(2)
                .splineToConstantHeading(new Vector2d(-34, -25), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-51.5,-17), Math.toRadians(180))
                //place pixel on right line
                .addTemporalMarker(pathTime -> pathTime-0.2,() -> {
                    preDropLeft.setPosition(0.95);
                })
                .build();
        //put pixel on left line

        TrajectorySequence WhiteStackOneLeft = robot.trajectorySequenceBuilder(PreDropLeft.end())
                .lineToLinearHeading(new Pose2d(-48,-13, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-53.5, -13), Math.toRadians(180))
                //pick up white pixel
                .addTemporalMarker(0.5, () -> {
                    //Close the preDrop servo
                    preDropLeft.setPosition(0.75);
                })
                .addTemporalMarker(pathTime -> pathTime - 2.8, () -> {
                    DIservo.setPosition(LiftConstants.StackMuncher1);
                })
                .addTemporalMarker(pathTime -> pathTime-2.5,() -> {
                    intake.setPower(1);
                    transfer.setPower(1);
                })
                .build();

        TrajectorySequence AprilTagLeft = robot.trajectorySequenceBuilder(WhiteStackOneLeft.end())
                .addTemporalMarker(0.5, () -> {
                    intake.setPower(-1);
                })
                .addTemporalMarker(2,()->{
                    intake.setPower(0);
                    transfer.setPower(0);
                })
                .lineTo(new Vector2d(32,-13))
                .splineToConstantHeading(new Vector2d(40,-36),Math.toRadians(0))

                .addTemporalMarker(pathTime -> pathTime - 1, () -> {
                    AprilTagVisionPortal.setProcessorEnabled(aprilProcessor, true);
                })
                //place pixel on backboard
                .addTemporalMarker(pathTime -> pathTime -0.5, () -> {
                    liftState = LiftState.CLOSE_PINCHERS;
                    storeLiftHeight = 700;
                })
                .build();

        TrajectorySequence BackBoardDropLeft = robot.trajectorySequenceBuilder(AprilTagLeft.end())
                .lineTo(new Vector2d(50, -36))
                .addTemporalMarker(pathTime -> pathTime -0.2, () -> {
                    readyToDrop = true;
                })
                .build();
        // pick up white pixels off stack
        TrajectorySequence WhiteStack2Left = robot.trajectorySequenceBuilder(BackBoardDropLeft.end())
                .splineToConstantHeading(new Vector2d(32, -9), Math.toRadians(180))
                .lineTo(new Vector2d(-55, -9))
                .addTemporalMarker(pathTime -> pathTime - 2.8, () -> {
                    DIservo.setPosition(LiftConstants.StackMuncher2);
                })
                .addTemporalMarker(pathTime -> pathTime-2.5,() -> {
                    intake.setPower(1);
                    transfer.setPower(1);
                })
                .build();

        TrajectorySequence AprilTagLeft2 = robot.trajectorySequenceBuilder(WhiteStack2Left.end())
                .addTemporalMarker(0.5, () -> {
                    intake.setPower(-1);
                    DIservo.setPosition(StackMuncherReturn);
                })
                .addTemporalMarker(2,()->{
                    intake.setPower(0);
                    transfer.setPower(0);
                })
                .lineTo(new Vector2d(32,-10))
                .splineToConstantHeading(new Vector2d(40,-28), Math.toRadians(0))
                .addTemporalMarker(pathTime -> pathTime -0.5, () -> {
//                    OpenCvVisionPortal.setProcessorEnabled(aprilProcessor, true);
                    liftState = LiftState.CLOSE_PINCHERS;
                    storeLiftHeight = 760;
                })
                .build();
        TrajectorySequence BackDropLeft2 = robot.trajectorySequenceBuilder(AprilTagLeft2.end())
                .lineTo(new Vector2d(50.5, -37))
                .addTemporalMarker(pathTime -> pathTime -0.2, () -> {
                    readyToDrop = true;
                })
                .build();

        TrajectorySequence ParkLeft = robot.trajectorySequenceBuilder(BackDropLeft2.end())
                .forward(2)
                .splineToConstantHeading(new Vector2d(50,-11), Math.toRadians(0))
                .build();


        TrajectorySequence ParkLeftUno = robot.trajectorySequenceBuilder(BackBoardDropLeft.end())
                .splineToConstantHeading(new Vector2d(50,-13), Math.toRadians(0))
                .build();

        TrajectorySequence PreDropRight = robot.trajectorySequenceBuilder(startPose)
                .back(2)
                .splineToConstantHeading(new Vector2d(-31.5, -26), Math.toRadians(0))
                .addTemporalMarker(pathTime -> pathTime-0.5,() -> {
                    preDropLeft.setPosition(0.95);
                })
                .turn(Math.toRadians(20))
                .build();

        TrajectorySequence WhiteStackOneRight = robot.trajectorySequenceBuilder(PreDropRight.end())
                .lineToLinearHeading(new Pose2d(-48,-13, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-53, -13.5), Math.toRadians(180))
                //pick up white pixel
                .addTemporalMarker(0.5, () -> {
                    //Close the preDrop servo
                    preDropLeft.setPosition(0.75);
                })
                .addTemporalMarker(pathTime -> pathTime - 2.8, () -> {
                    DIservo.setPosition(LiftConstants.StackMuncher1);
                })
                .addTemporalMarker(pathTime -> pathTime-2.5,() -> {
                    intake.setPower(1);
                    transfer.setPower(1);
                })
                .build();

        TrajectorySequence AprilTagRight = robot.trajectorySequenceBuilder(WhiteStackOneLeft.end())
                .addTemporalMarker(0.5, () -> {
                    intake.setPower(-1);
                })
                .addTemporalMarker(2,()->{
                    intake.setPower(0);
                    transfer.setPower(0);
                })
                .lineTo(new Vector2d(32,-13))
                .splineToConstantHeading(new Vector2d(40,-36),Math.toRadians(0))

                .addTemporalMarker(pathTime -> pathTime-1, () -> {
                   AprilTagVisionPortal.setProcessorEnabled(aprilProcessor, true);
                })
                //place pixel on backboard
                .addTemporalMarker(pathTime -> pathTime -0.5, () -> {
                    liftState = LiftState.CLOSE_PINCHERS;
                    alternateWristAngle = LiftConstants.wristMiddle1;
                    storeLiftHeight = 700;
                })
                .build();

        TrajectorySequence BackBoardDropRight = robot.trajectorySequenceBuilder(AprilTagRight.end())
                .lineTo(new Vector2d(49.5, -46))
                .addTemporalMarker(pathTime -> pathTime -0.2, () -> {
                    readyToDrop = true;
                })
                .build();

        TrajectorySequence WhiteStack2Right = robot.trajectorySequenceBuilder(BackBoardDropRight.end())
                .splineToConstantHeading(new Vector2d(32, -9), Math.toRadians(180))
                .lineTo(new Vector2d(-55, -9))
                .addTemporalMarker(pathTime -> pathTime - 2.8, () -> {
                    DIservo.setPosition(LiftConstants.StackMuncher2);
                })
                .addTemporalMarker(pathTime -> pathTime-2.5,() -> {
                    intake.setPower(1);
                    transfer.setPower(1);
                })
                .build();

        TrajectorySequence AprilTagRight2 = robot.trajectorySequenceBuilder(WhiteStack2Right.end())
                .addTemporalMarker(0.5, () -> {
                    intake.setPower(-1);
                    DIservo.setPosition(StackMuncherReturn);
                })
                .addTemporalMarker(2,()->{
                    intake.setPower(0);
                    transfer.setPower(0);
                })
                .lineTo(new Vector2d(32,-10))
                .splineToConstantHeading(new Vector2d(40,-28), Math.toRadians(0))
                .addTemporalMarker(pathTime -> pathTime -0.5, () -> {
//                    OpenCvVisionPortal.setProcessorEnabled(aprilProcessor, true);
                    liftState = RedLeftAsync.LiftState.CLOSE_PINCHERS;
                    alternateWristAngle = LiftConstants.wristMiddle2;
                    storeLiftHeight = 760;
                })
                .build();
        TrajectorySequence BackDropRight2 = robot.trajectorySequenceBuilder(AprilTagRight2.end())
                .lineTo(new Vector2d(50.5, -30))
                .addTemporalMarker(pathTime -> pathTime -0.2, () -> {
                    readyToDrop = true;
                })
                .build();


        // pick up white pixels off stack
        TrajectorySequence ParkRight = robot.trajectorySequenceBuilder(BackDropRight2.end())
                .splineToConstantHeading(new Vector2d(50,-11), Math.toRadians(0))
                .build();

        TrajectorySequence ParkRightUno = robot.trajectorySequenceBuilder(BackBoardDropRight.end())
                .splineToConstantHeading(new Vector2d(50,-13), Math.toRadians(0))
                .build();


        TrajectorySequence PreDropMid = robot.trajectorySequenceBuilder(startPose)
                .back(2)
                .splineToConstantHeading(new Vector2d(-48, -25), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-44,-15), Math.toRadians(0))
                //place pixel on right line
                .addTemporalMarker(pathTime -> pathTime-0.2,() -> {
                    preDropLeft.setPosition(0.95);
                })

                .build();
        TrajectorySequence WhiteStackOneMid = robot.trajectorySequenceBuilder(PreDropMid.end())
                .lineToLinearHeading(new Pose2d(-48,-13, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-53, -13), Math.toRadians(180))
                //pick up one white pixel
//                .addTemporalMarker(pathTime -> pathTime-0.2,() -> {
//                    preDropRight.setPosition(0.85);
//                })
                .addTemporalMarker(0.5, () -> {
                    //Close the preDrop servo
                    preDropLeft.setPosition(0.75);
                })
                .addTemporalMarker(pathTime -> pathTime - 2.2, () -> {
                    DIservo.setPosition(LiftConstants.StackMuncher1);
                })
                .addTemporalMarker(pathTime -> pathTime-2.5,() -> {
                    intake.setPower(1);
                    transfer.setPower(1);
                })
                .build();

        TrajectorySequence AprilTagMid = robot.trajectorySequenceBuilder(WhiteStackOneMid.end())
                .waitSeconds(0.5)
                .forward(2)
                .addTemporalMarker(0.5, () -> {
                    intake.setPower(-1);
                })
                .addTemporalMarker(1.5,()->{
                    intake.setPower(0);
                    transfer.setPower(0);
                })
                .lineTo(new Vector2d(16,-13))
                .splineToConstantHeading(new Vector2d(40,-36),Math.toRadians(0))
                //50
                //place pixel on backboard
                .addTemporalMarker(pathTime -> pathTime - 0.5, () -> {
                    AprilTagVisionPortal.setProcessorEnabled(aprilProcessor, true);
                    liftState = LiftState.CLOSE_PINCHERS;
                    storeLiftHeight = 700;
                })
                .build();

        TrajectorySequence BackBoardDropMid = robot.trajectorySequenceBuilder(AprilTagMid.end())
                .lineTo(new Vector2d(49.5   , -43))
                .addTemporalMarker(pathTime -> pathTime -0.2, () -> {
                    readyToDrop = true;
                })
                .build();

        TrajectorySequence WhiteStack2Mid = robot.trajectorySequenceBuilder(BackBoardDropMid.end())
                .splineToConstantHeading(new Vector2d(32, -9), Math.toRadians(180))
                .lineTo(new Vector2d(-55, -9))
                .addTemporalMarker(pathTime -> pathTime - 2.8, () -> {
                    DIservo.setPosition(LiftConstants.StackMuncher2);
                })
                .addTemporalMarker(pathTime -> pathTime-2.5,() -> {
                    intake.setPower(1);
                    transfer.setPower(1);
                })
                .build();

        TrajectorySequence AprilTagMid2 = robot.trajectorySequenceBuilder(WhiteStack2Mid.end())
                .addTemporalMarker(0.5, () -> {
                    intake.setPower(-1);
                    DIservo.setPosition(StackMuncherReturn);
                })
                .addTemporalMarker(2,()->{
                    intake.setPower(0);
                    transfer.setPower(0);
                })
                .lineTo(new Vector2d(32,-10))
                .splineToConstantHeading(new Vector2d(40,-28), Math.toRadians(0))
                .addTemporalMarker(pathTime -> pathTime -0.5, () -> {
//                    OpenCvVisionPortal.setProcessorEnabled(aprilProcessor, true);
                    liftState = RedLeftAsync.LiftState.CLOSE_PINCHERS;
                    alternateWristAngle = LiftConstants.wristMiddle2;
                    storeLiftHeight = 760;
                })
                .build();
        TrajectorySequence BackDropMid2 = robot.trajectorySequenceBuilder(AprilTagMid2.end())
                .lineTo(new Vector2d(50.5, -30))
                .addTemporalMarker(pathTime -> pathTime -0.2, () -> {
                    readyToDrop = true;
                })
                .build();

        TrajectorySequence ParkMid = robot.trajectorySequenceBuilder(BackDropMid2.end())
                .splineToConstantHeading(new Vector2d(50,-11), Math.toRadians(0))
                .build();

        TrajectorySequence ParkMidUno = robot.trajectorySequenceBuilder(BackBoardDropMid.end())
                .splineToConstantHeading(new Vector2d(50,-13), Math.toRadians(0))
                .build();



        //IMU setup
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(RevOrientation));
        imu.resetYaw();

        //Assign both webcams
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");

        //Lift initialization
        leftLift = hardwareMap.get(DcMotor.class, "left_lift");
        lift.initAuto(hardwareMap);
        transfer = hardwareMap.get(CRServo.class, "transfer");
        preDropLeft = hardwareMap.get(Servo.class,  "preDropLeft");
        DIservo = hardwareMap.get(Servo.class, "DIservo");
        intake = hardwareMap.get(DcMotor.class, "intake");
        frontPincher = hardwareMap.get(Servo.class, "front_pincher");
        backPincher = hardwareMap.get(Servo.class, "back_pincher");
        wrist = hardwareMap.get(Servo.class, "wrist_servo");
        boxBeam = hardwareMap.get(DigitalChannel.class, "box_beam");
        Blinky = hardwareMap.get(RevBlinkinLedDriver.class, "Blinky");


        //init AprilTag & Colour processor
        aprilProcessor = new AprilTagProcessor.Builder()
                .build();
        redProcessorFrontStage = new RedProcessorFrontStage(telemetry);
int[] ids = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);
        //   blueProcessor = new BlueProcessor(telemetry);

        //init VisionPortal
        OpenCvVisionPortal = new VisionPortal.Builder()
                .setCamera(webcam2)
                .addProcessor(redProcessorFrontStage)
             //   .addProcessor(aprilProcessor)
                //   .addProcessor(blueProcessor)
                .setLiveViewContainerId(ids[0])
                .setCameraResolution(new Size(1920,1080))
                // .enableLiveView(true)
                .build();
        OpenCvVisionPortal.setProcessorEnabled(redProcessorFrontStage, true);
      //  OpenCvVisionPortal.setProcessorEnabled(aprilProcessor, false);
        //Updates telemetry with current prop location
        while (opModeInInit()){
            telemetry.addData("Location: ", redProcessorFrontStage.getLocation());
            //   telemetry.addData("Location: ", blueProcessor.getLocation());
            telemetry.update();
        }
        waitForStart();
      RedProcessorFrontStage.Location location = redProcessorFrontStage.getLocation();
        OpenCvVisionPortal.setProcessorEnabled(redProcessorFrontStage, false);
        sleep(14000);

        AprilTagVisionPortal = new VisionPortal.Builder()
                .setCamera(webcam2)
               // .addProcessor(redProcessorFrontStage)
                   .addProcessor(aprilProcessor)
                //   .addProcessor(blueProcessor)
                .setLiveViewContainerId(ids[1])
                .setCameraResolution(new Size(800,600))
             //   .enableLiveView(true)
                .build();
        //Sets beginning trajectory and drive state based off the blueProcessor's detection
        switch (location) {
            // switch (blueProcessor.getLocation()) {

            case LEFT:
            case NOT_FOUND:
                robot.followTrajectorySequenceAsync(PreDropLeft);
                break;
            case MIDDLE:
                robot.followTrajectorySequenceAsync(PreDropMid);
                break;
            case RIGHT:
                robot.followTrajectorySequenceAsync(PreDropRight);
                break;
        }

        imuTimer.reset();
        Blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

        DIservo.setPosition(StackMuncherReturn);

        //Close pincher for yellow preload
        backPincher.setPosition(backPincherClose);

        //Turn off Vision Portal to conserve resources
       //  OpenCvVisionPortal.setProcessorEnabled(redProcessorFrontStage, false);
        //Need to test if this prevents crashing when stopping autonomous
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            //Updates RoadRunner's trajectory following
            robot.update();

            //Chaining async trajectories through state machine
            //State names describe what the robot is doing while in that state, not what the state contains
            switch (driveState) {
                case PREDROP:
                    //Wait for predrop trajectory to finish
                    if (!robot.isBusy()) {
                        //Runs the trajectory to the white stack, based off end position
                        switch (redProcessorFrontStage.getLocation()) {
                            case LEFT:
                            case NOT_FOUND:
                                robot.followTrajectorySequenceAsync(WhiteStackOneLeft);
                                break;
                            case MIDDLE:
                                robot.followTrajectorySequenceAsync(WhiteStackOneMid);
                                break;
                            case RIGHT:
                                robot.followTrajectorySequenceAsync(WhiteStackOneRight);
                                break;
                        }
                        //Advances driveState to next trajectory
                        driveState = State.APRIL_TAG;
                        robot.setPoseEstimate(new Pose2d(poseEstimate.getX(), poseEstimate.getY(), imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)-Math.toRadians(90)));
                    }
                    break;
                case APRIL_TAG:
                    //Intake one from the middle stack
                    if (!robot.isBusy()) {
                        //Based on camera detection from beginning, run trajectory from stack to backdrop
                        switch (redProcessorFrontStage.getLocation()) {
                            case LEFT:
                            case NOT_FOUND:
                                robot.followTrajectorySequenceAsync(AprilTagLeft);
                                break;
                            case MIDDLE:
                                robot.followTrajectorySequenceAsync(AprilTagMid);
                                break;
                            case RIGHT:
                                robot.followTrajectorySequenceAsync(AprilTagRight);
                                break;
                        }
                        //Bring back up the stack muncher right at the beginning of the trajectory
                        DIservo.setPosition(StackMuncherReturn);
                        driveState = State.BACKBOARD_DROP;
                        robot.setPoseEstimate(new Pose2d(poseEstimate.getX(), poseEstimate.getY(), imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)-Math.toRadians(90)));
                    }
                    break;
                case BACKBOARD_DROP:
                    if (!robot.isBusy()) {
                        List<AprilTagDetection> currentDetections = aprilProcessor.getDetections();
                        for (AprilTagDetection detection : currentDetections) {
                            if (detection.id == 4) {

                                // calculation stuff
                                double corrected_range = Math.sqrt(Math.pow(detection.ftcPose.y+y_offset,2) + Math.pow(detection.ftcPose.x,2));
                                double theta_rad = Math.toRadians(detection.ftcPose.yaw - detection.ftcPose.bearing);
                                x_prime = Math.sin(theta_rad) * corrected_range;
                                y_prime = Math.cos(theta_rad) * corrected_range;
                                robot.setPoseEstimate(new Pose2d(aprilTagLibrary[1].getX() - y_prime, aprilTagLibrary[1].getY() + detection.ftcPose.x, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - Math.toRadians(90)));
                                Blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
                                telemetry.addLine(String.format("x y %6.1f %6.1f", detection.ftcPose.x, detection.ftcPose.y));
                            }
                        }
                        //Park after placing the pixels
                        switch (redProcessorFrontStage.getLocation()) {
                            case LEFT:
                            case NOT_FOUND:
                                robot.followTrajectorySequenceAsync(BackBoardDropLeft);
                                break;
                            case MIDDLE:
                                robot.followTrajectorySequenceAsync(BackBoardDropMid);
                                break;
                            case RIGHT:
                                robot.followTrajectorySequenceAsync(BackBoardDropRight);
                                break;
                        }
                       driveState = State.BACKBOARD_DROP2;
                    }
                    break;
                case BACKBOARD_DROP2:
                    if (!robot.isBusy()) {
                        if (ExtraCycle) {
                            switch (redProcessorFrontStage.getLocation()) {
                                case LEFT:
                                case NOT_FOUND:
                                    robot.followTrajectorySequenceAsync(WhiteStack2Left);
                                    break;
                                case MIDDLE:
                                    robot.followTrajectorySequenceAsync(WhiteStack2Mid);
                                    break;
                                case RIGHT:
                                    robot.followTrajectorySequenceAsync(WhiteStack2Right);
                                    break;
                            }
                            driveState = State.IDLE;
                            robot.setPoseEstimate(new Pose2d(poseEstimate.getX(), poseEstimate.getY(), imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - Math.toRadians(90)));
                        } else {
                            switch (redProcessorFrontStage.getLocation()) {
                                case LEFT:
                                case NOT_FOUND:
                                    robot.followTrajectorySequenceAsync(ParkLeftUno);
                                    break;
                                case MIDDLE:
                                    robot.followTrajectorySequenceAsync(ParkMidUno);
                                    break;
                                case RIGHT:
                                    robot.followTrajectorySequenceAsync(ParkRightUno);
                                    break;
                            }
                            driveState = State.IDLE;
                            robot.setPoseEstimate(new Pose2d(poseEstimate.getX(), poseEstimate.getY(), imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - Math.toRadians(90)));
                        }
                    }
                    break;
            }
            telemetry.addLine(String.format("CX CY %6.1f %6.1f", x_prime, y_prime));


            //State machine for controlling lift
            //Is activated when the liftState is changed to CLOSE_PINCHERS
            //Drops when readyToDrop is set to true
            switch (liftState) {
                case LIFT_IDLE:
                    liftTimer.reset();
                    break;
                case CLOSE_PINCHERS:
                    //First close front pincher
                    frontPincher.setPosition(frontPincherClose);

                    //After x seconds close back pincher
                    if (liftTimer.seconds() > 0.3){
                        backPincher.setPosition(backPincherClose);
                    }
                    //Wait for back pincher to close before extending lift
                    if (liftTimer.seconds() > 0.6) {
                        liftHeight = storeLiftHeight;
                        liftState = LiftState.LIFT_EXTEND;
                    }
                    break;
                case LIFT_EXTEND:
                    //Check if lift has fully extended
                    if (leftLift.getCurrentPosition() > 400) {
                        //Deploy box
                        lift.extendBox();
                        //Rotate wrist to Middle1 unless other angle has been specified
                        if (alternateWristAngle != 0)
                            wrist.setPosition(alternateWristAngle);
                        else
                            wrist.setPosition(LiftConstants.wristMiddle2);

                        liftState = LiftState.BOX_EXTEND;
                        liftTimer.reset();
                    }
                    break;
                case BOX_EXTEND:
                    //Wait for robot to reach position
                    if (readyToDrop) {
                        //Release pixels and move to next state
                        liftState = LiftState.LIFT_DUMP;
                        frontPincher.setPosition(frontPincherOpen);
                        backPincher.setPosition(backPincherOpen);
                        liftTimer.reset();
                    }
                    break;
                case LIFT_DUMP:
                    //Wait x seconds for robot to drive away from backboard
                    if(liftTimer.seconds() >= dropTime) {
                        //Retract box and wrist
                        lift.retractBox();
                        liftTimer.reset();
                        wrist.setPosition(wristIdle);
                        alternateWristAngle = 0;
                        liftState = LiftState.BOX_RETRACT;
                    }
                    break;
                case BOX_RETRACT:
                    // Wait for servo to return to Idle
                    if (liftTimer.seconds() >= 0.15) {
                        liftState = LiftState.LIFT_RETRACT;
                        //Retract Lift
                        liftHeight = liftRetracted;
                    }
                    break;
                case LIFT_RETRACT:
                    //Wait for Lift to return to idle
                    if (Math.abs(leftLift.getCurrentPosition() - liftRetracted) < 10) {
                        liftState = LiftState.LIFT_IDLE;
                        readyToDrop = false;
                    }
                    break;
                default:
                    //Should never happen but just in case
                    liftState = LiftState.LIFT_IDLE;
            }
            telemetry.addData("Lift State", liftState);
            //Updates lift PID control with current liftHeight variable
            lift.setHeight(liftHeight);

            //stores the current robots position into a pose
            poseEstimate = robot.getPoseEstimate();
            u = poseEstimate.getHeading();
            x = xPrevious + u;
            p = pPrevious + Q;
            K = p/(p + R);
            z = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + Math.toRadians(90);
            x = x + K * (z-x);
            p = (1 - K) * p;
            xPrevious = x;
            pPrevious = p;
            telemetry.addData("RR heading:", poseEstimate.getHeading());
            telemetry.addData("Kalman Heading:", x);
            telemetry.addData("P", p);
            telemetry.addData("K", K);
            telemetry.addData("IMU Heading: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            //Updates heading every x seconds using the imu
            if(imuTimer.seconds() >= headingInterval) {
                //Keeps robots x and y, but assigns a new heading
                //robot.setPoseEstimate(new Pose2d(poseEstimate.getX(), poseEstimate.getY(), imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)-Math.toRadians(90)));
                imuTimer.reset();
            }
            telemetry.addData("Box Beam", boxBeam.getState());
            telemetry.update();
        }
    }
}


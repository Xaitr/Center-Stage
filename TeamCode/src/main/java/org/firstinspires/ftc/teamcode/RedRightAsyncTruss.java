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
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous
public class RedRightAsyncTruss extends LinearOpMode {

    //This enum defines steps of the trajectories
     private enum State {
        BACKBOARD_DROP, //First drop of the yellow preload on the backboard
        PREDROP, //Then place the purple preload on the spikemark
        GENERAL_STACK, //Then drive to the white stacks
        INTAKE_STACK, //Wait for pixels to intake
        BACKBOARD_STACK, //Drive to backstage to place white pixels
        GENERAL_STACK2,
        INTAKE_STACK2, //Drive to stack from the backstage
        BACKBOARD_STACK2, //Drive to backstage to place white pixels
        IDLE //Robot enters idle when finished
    }

    private IMU imu = null;
    private DigitalChannel boxBeam = null;
    Pose2d poseEstimate;

    //Instantiate our driveState
    State driveState = State.BACKBOARD_DROP;

    //Declare purple preload servo
    private Servo preDropRight = null;

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

    //Timer for in between trajectories
    ElapsedTime driveTimer = new ElapsedTime();

    //Timer for periodically updating the heading with the imu
    ElapsedTime imuTimer = new ElapsedTime();
    private double headingInterval = 5;

    //Lets the lift state machine know when the trajectory to the backboard is finished
    private boolean readyToDrop = false;

    //Time for robot to drive away and clear the backboard
    private double dropTime = 1;

    //Declare our April tag & OpenCV vision portal
    private VisionPortal OpenCvVisionPortal;

    //Declare both webcams
    private WebcamName webcam1, webcam2;

    //Declare our AprilTag Processor
    private AprilTagProcessor aprilProcessor;

    //Declare our colour processor
    private RedProcessor redProcessor;

    @Override
    public void runOpMode() throws InterruptedException {
        //Instantiate SampleMecanumDrive
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        //Define robots starting position and orientation
        Pose2d startPose = new Pose2d(13, -65, Math.toRadians(180));
        robot.setPoseEstimate(startPose);

        TrajectorySequence BackBoardDropLeft = robot.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(53.5,-31.5))
                // place pixel on backboard
                .addTemporalMarker(pathTime -> pathTime-1.5,() -> {
                    //Starts extending lift x seconds before reaching the backboard
                    liftState = LiftState.CLOSE_PINCHERS;
                    storeLiftHeight = LiftConstants.liftAuto;
                })
                .addTemporalMarker(pathTime -> pathTime-0.2,() -> {
                    readyToDrop = true;
                })
                .build();
        TrajectorySequence PreDropLeft = robot.trajectorySequenceBuilder(BackBoardDropLeft.end())
                .lineTo(new Vector2d(10.5, -42))
                .addTemporalMarker(pathTime -> pathTime-0.2,() -> {
                    preDropRight.setPosition(0.55);
                })
                //place pixel on line
                .build();

        TrajectorySequence ParkLeft = robot.trajectorySequenceBuilder(PreDropLeft.end())
                .lineTo(new Vector2d(50, -60))
                .addTemporalMarker(0.5, () -> {
                    //Close the preDrop servo
                    preDropRight.setPosition(0.75);
                })
                .build();

        TrajectorySequence WhiteStackOneLeft = robot.trajectorySequenceBuilder(PreDropLeft.end())
                .lineTo(new Vector2d(20, -50))
                .addTemporalMarker(0.5, () -> {
                    //Close the preDrop servo
                    preDropRight.setPosition(0.75);
                })
                .splineToConstantHeading(new Vector2d (10,-60), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d (-22,-59.5), Math.toRadians(180))
                .addTemporalMarker(pathTime -> pathTime - 3, () -> {
                    DIservo.setPosition(LiftConstants.StackMuncher1);
                })
                .addTemporalMarker(pathTime -> pathTime-2.5,() -> {
                    intake.setPower(1);
                    transfer.setPower(1);
                })
                .splineTo(new Vector2d(-52,-51),Math.toRadians(145))
                .forward(4)
                .back(3)
                .addDisplacementMarker(() -> {
                    DIservo.setPosition(LiftConstants.StackMuncher2);
                    intake.setPower(1);

                })
//                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {
//                    intake.setPower(-1);
//                })
                .forward(2.5)
                // pick up white pixels off stack
                .build();
        TrajectorySequence BackDropLeft = robot.trajectorySequenceBuilder(WhiteStackOneLeft.end())
                .forward(2)
                .lineToLinearHeading(new Pose2d(-35,-62,  Math.toRadians(-180)))
                .lineTo(new Vector2d(46,-63))

                .addTemporalMarker(1.2, () -> {
                    //Reject any extra pixel that might have been intaked
                    intake.setPower(-1);
                    transfer.setPower(1);
                })
                .addTemporalMarker(2.5,() -> {
                    //Turn off the intake
                    intake.setPower(0);
                    transfer.setPower(0);

                })
                //place two white pixels
                //put pixel on backboard
                .addTemporalMarker(pathTime -> pathTime-2,() -> {
                    //Starts extending lift x seconds before reaching the backboard
                    liftState = LiftState.CLOSE_PINCHERS;
                    storeLiftHeight = LiftConstants.liftAuto;
                })
                .addTemporalMarker(pathTime -> pathTime-0.5,() -> {
                    readyToDrop = true;
                })
                .build();

        TrajectorySequence BackBoardDropRight = robot.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(53.5,-44))
                //put pixel on backboard
                .addTemporalMarker(pathTime -> pathTime-1.5,() -> {
                    //Starts extending lift x seconds before reaching the backboard
                    liftState = LiftState.CLOSE_PINCHERS;
                    storeLiftHeight = LiftConstants.liftAuto;
                })
                .addTemporalMarker(pathTime -> pathTime-0.2,() -> {
                    readyToDrop = true;
                })
                .build();
        TrajectorySequence PreDropRight = robot.trajectorySequenceBuilder(BackBoardDropRight.end())
                .lineTo(new Vector2d(33, -40))
                .addTemporalMarker(pathTime -> pathTime-0.2,() -> {
                    preDropRight.setPosition(0.55);
                })
                .build();

        TrajectorySequence ParkRight = robot.trajectorySequenceBuilder(PreDropRight.end())
                .lineTo(new Vector2d(50, -60))
                .addTemporalMarker(0.5, () -> {
                    //Close the preDrop servo
                    preDropRight.setPosition(0.75);
                })
                .build();

        TrajectorySequence WhiteStackOneRight = robot.trajectorySequenceBuilder(PreDropRight.end())
                .lineTo(new Vector2d(33, -51))
                .addTemporalMarker(0.5, () -> {
                    //Close the preDrop servo
                    preDropRight.setPosition(0.75);
                })
                .splineToConstantHeading(new Vector2d (10,-58.5), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d (-20,-59.5), Math.toRadians(180))
                .addTemporalMarker(pathTime -> pathTime - 3, () -> {
                    DIservo.setPosition(LiftConstants.StackMuncher1);
                })
                .addTemporalMarker(pathTime -> pathTime-2.5,() -> {
                    intake.setPower(1);
                    transfer.setPower(1);
                })
                .splineTo(new Vector2d(-51,-47),Math.toRadians(160))
                .forward(3.5)
                .back(3)
                .addDisplacementMarker(() -> {
                    DIservo.setPosition(LiftConstants.StackMuncher2);
                    intake.setPower(-1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    intake.setPower(1);
                })
                .forward(2.5)
                .build();

        TrajectorySequence BackDropRight = robot.trajectorySequenceBuilder(WhiteStackOneRight.end())
                .forward(2)
                .lineToLinearHeading(new Pose2d(-35,-62.5,  Math.toRadians(-180)))
                .lineTo(new Vector2d(50,-64))

                .addTemporalMarker(1.2, () -> {
                    //Reject any extra pixel that might have been intaked
                    intake.setPower(-1);
                    transfer.setPower(1);
                })
                .addTemporalMarker(2.5,() -> {
                    //Turn off the intake
                    intake.setPower(0);
                    transfer.setPower(0);

                })
                //place two white pixels
                //put pixel on backboard
                .addTemporalMarker(pathTime -> pathTime-2,() -> {
                    //Starts extending lift x seconds before reaching the backboard
                    liftState = LiftState.CLOSE_PINCHERS;
                    storeLiftHeight = LiftConstants.liftAuto;
                })
                .addTemporalMarker(pathTime -> pathTime-0.5,() -> {
                    readyToDrop = true;
                })
                .build();



        TrajectorySequence BackBoardDropMid = robot.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(53.5,-38))
                // place pixel on backboard
                .addTemporalMarker(pathTime -> pathTime-1.5,() -> {
                    //Starts extending lift
                    liftState = LiftState.CLOSE_PINCHERS;
                    storeLiftHeight = LiftConstants.liftAuto;
                })
                .addTemporalMarker(pathTime -> pathTime-0.2,() -> {
                    readyToDrop = true;
                })
                .build();
        TrajectorySequence PreDropMid = robot.trajectorySequenceBuilder(BackBoardDropMid.end())
                .lineTo(new Vector2d(26, -32))
                .addTemporalMarker(pathTime -> pathTime-0.2,() -> {
                    preDropRight.setPosition(0.55);
                })
                .build();

        TrajectorySequence ParkMid = robot.trajectorySequenceBuilder(PreDropMid.end())
                .lineTo(new Vector2d(50, -60))
                .addTemporalMarker(0.5, () -> {
                    //Close the preDrop servo
                    preDropRight.setPosition(0.75);
                })
                .build();

        TrajectorySequence WhiteStackOneMid = robot.trajectorySequenceBuilder(PreDropMid.end())
                .lineTo(new Vector2d(20, -53))
                .addTemporalMarker(0.5, () -> {
                    //Close the preDrop servo
                    preDropRight.setPosition(0.75);
                })
                .splineToConstantHeading(new Vector2d (10,-61), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d (-15,-60), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d (-20,-60), Math.toRadians(180))
                .addTemporalMarker(pathTime -> pathTime - 3, () -> {
                    DIservo.setPosition(LiftConstants.StackMuncher1);
                })
                .addTemporalMarker(pathTime -> pathTime-2.5,() -> {
                    intake.setPower(1);
                    transfer.setPower(1);
                })
                .splineTo(new Vector2d(-51,-47),Math.toRadians(160))
                .forward(3.5)
                .back(3)
                .addDisplacementMarker(() -> {
                    DIservo.setPosition(LiftConstants.StackMuncher2);
                    intake.setPower(1);
                })
//                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {
//                    intake.setPower(-1);
//                })
                .forward(2.5)
                .build();

        TrajectorySequence BackDrop = robot.trajectorySequenceBuilder(WhiteStackOneMid.end())
                .forward(2)
                .lineToLinearHeading(new Pose2d(-35,-61.5,  Math.toRadians(-180)))
                .lineTo(new Vector2d(50,-63.5))

                .addTemporalMarker(1.2, () -> {
                    //Reject any extra pixel that might have been intaked
                    intake.setPower(-1);
                    transfer.setPower(1);
                })
                .addTemporalMarker(1.8,() -> {
                    //Turn off the intake
                    intake.setPower(0);
                    transfer.setPower(0);

                })
                //place two white pixels
                //put pixel on backboard
                .addTemporalMarker(pathTime -> pathTime-2,() -> {
                    //Starts extending lift x seconds before reaching the backboard
                    liftState = LiftState.CLOSE_PINCHERS;
                    storeLiftHeight = LiftConstants.liftAuto;
                })
                .addTemporalMarker(pathTime -> pathTime-0.5,() -> {
                    readyToDrop = true;
                })
                .build();
        //Moves from stack to the backdrop
        TrajectorySequence WhiteStackTwo = robot.trajectorySequenceBuilder(BackDrop.end())
                .splineToConstantHeading(new Vector2d (10,-60), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d (-20,-60), Math.toRadians(180))
                .splineTo(new Vector2d(-49, -37), Math.toRadians(200))
                .forward(3)
                //Turn on intake and lower the stack-muncher
                .addTemporalMarker(pathTime -> pathTime - 3, () -> {
                    DIservo.setPosition(LiftConstants.StackMuncher3);
                })
                .addTemporalMarker(pathTime -> pathTime-2.5,() -> {
                    intake.setPower(1);
                    transfer.setPower(1);
                })
                .back(3)
                .addDisplacementMarker(() -> {
                    DIservo.setPosition(LiftConstants.StackMuncher4);
                    intake.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {
                    intake.setPower(-1);
                })
                .forward(3)
                .build();

        TrajectorySequence BackDrop2 = robot.trajectorySequenceBuilder(WhiteStackOneMid.end())
                .forward(4)
                .lineToLinearHeading(new Pose2d(-35,-52,  Math.toRadians(-180)))
                .lineTo(new Vector2d(50,-52), SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 1.5))

                .addTemporalMarker(1.2, () -> {
                    //Reject any extra pixel that might have been intaked
                    intake.setPower(-1);
                    transfer.setPower(1);
                })
                .addTemporalMarker(1.8,() -> {
                    //Turn off the intake
                    intake.setPower(0);
                    transfer.setPower(0);

                })
                //place two white pixels
                //put pixel on backboard
                .addTemporalMarker(pathTime -> pathTime-2,() -> {
                    //Starts extending lift x seconds before reaching the backboard
                    liftState = LiftState.CLOSE_PINCHERS;
                    storeLiftHeight = LiftConstants.liftAuto;
                })
                .addTemporalMarker(pathTime -> pathTime-0.5,() -> {
                    readyToDrop = true;
                })
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
        preDropRight = hardwareMap.get(Servo.class,  "preDropRight");
        DIservo = hardwareMap.get(Servo.class, "DIservo");
        intake = hardwareMap.get(DcMotor.class, "intake");
        frontPincher = hardwareMap.get(Servo.class, "front_pincher");
        backPincher = hardwareMap.get(Servo.class, "back_pincher");
        wrist = hardwareMap.get(Servo.class, "wrist_servo");
        boxBeam = hardwareMap.get(DigitalChannel.class, "box_beam");

        //initAuto AprilTag & Colour processor
        aprilProcessor = new AprilTagProcessor.Builder()
                .build();
        redProcessor = new RedProcessor(telemetry);

        //initAuto VisionPortal
        OpenCvVisionPortal = new VisionPortal.Builder()
                .setCamera(webcam1)
                .addProcessor(redProcessor)
                .setCameraResolution(new Size(1920,1080))
                .enableLiveView(true)
                .build();

        //Updates telemetry with current prop location
        while (opModeInInit()){
            telemetry.addData("Location: ", redProcessor.getLocation());
            telemetry.update();
        }
        waitForStart();

        //Sets beginning trajectory and drive state based off the redProcessor's detection
        switch (redProcessor.getLocation()) {
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

        imuTimer.reset();

        DIservo.setPosition(StackMuncherReturn);

        //Close pincher for yellow preload
        backPincher.setPosition(backPincherClose);

        //Turn off Vision Portal to conserve resources
        OpenCvVisionPortal.stopStreaming();

        //Need to test if this prevents crashing when stopping autonomous
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            //Updates RoadRunner's trajectory following
            robot.update();

            //Chaining async trajectories through state machine
            switch (driveState) {
                case BACKBOARD_DROP:
                    //Wait for backboard trajectory to finish
                    if (!robot.isBusy()) {
                        robot.setPoseEstimate(new Pose2d(poseEstimate.getX(), poseEstimate.getY(), imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)+3.14159));

                        //Runs the the different preDrop purple trajectories based on camera detection
                        switch (redProcessor.getLocation()) {
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
                        //Advances driveState to next trajectory
                        driveState = State.PREDROP;
                    }
                    break;
                case PREDROP:
                    if (!robot.isBusy()) {
                        //Drop preloaded purple pixel
                        robot.setPoseEstimate(new Pose2d(poseEstimate.getX(), poseEstimate.getY(), imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)+3.14159));

                        //Based on camera detection from beginning, run trajectory from spike mark to stacks
                        switch (redProcessor.getLocation()) {
                            case LEFT:
                            case NOT_FOUND:
                                robot.followTrajectorySequenceAsync(ParkLeft);
                                break;
                            case MIDDLE:
                                robot.followTrajectorySequenceAsync(ParkMid);
                                break;
                            case RIGHT:
                                robot.followTrajectorySequenceAsync(ParkRight);
                                break;
                        }
                        driveState = State.IDLE;
                    }
                    break;
                case GENERAL_STACK:
                    if (!boxBeam.getState())
                        DIservo.setPosition(LiftConstants.StackMuncher2);

                    if (!robot.isBusy()) {
                        //Reset timer once we've reached the stacks
                        robot.setPoseEstimate(new Pose2d(poseEstimate.getX(), poseEstimate.getY(), imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)+3.14159));
                        driveTimer.reset();
                        driveState = State.INTAKE_STACK;
                    }
                    break;
                case INTAKE_STACK:
                    //Lower the Stack muncher for the second pixel after x seconds


                    //Drive to backstage after x seconds
                    if (driveTimer.seconds() >= 0.3) {
                        DIservo.setPosition(LiftConstants.StackMuncherReturn);
                        //Bring back the Stack Muncher to idle
                    }
                    if (driveTimer.seconds() >= 1){
                        driveState = State.BACKBOARD_STACK;
                        switch (redProcessor.getLocation()) {
                            case LEFT:
                            case NOT_FOUND:
                                robot.followTrajectorySequenceAsync(BackDropLeft);
                                break;
                            case MIDDLE:
                                robot.followTrajectorySequenceAsync(BackDrop);
                                break;
                            case RIGHT:
                                robot.followTrajectorySequenceAsync(BackDropRight);
                                break;
                        }
                    }
                    break;
                case BACKBOARD_STACK:
//                    if (!robot.isBusy()) {
//                        telemetry.addData("UH OH ", "AHHH");
//                        robot.followTrajectorySequenceAsync(WhiteStackTwo);
//                        driveTimer.reset();
//                        driveState = State.GENERAL_STACK2;
//                    }
                    break;
                case GENERAL_STACK2:
                    if (!boxBeam.getState())
                        DIservo.setPosition(LiftConstants.StackMuncher4);

                    if (!robot.isBusy()) {
                        //Reset timer once we've reached the stacks
                        driveTimer.reset();
                        driveState = State.INTAKE_STACK2;
                    }
                    break;
                case INTAKE_STACK2:
                    if (driveTimer.seconds() >= 0.3) {
                        DIservo.setPosition(LiftConstants.StackMuncherReturn);
                        //Bring back the Stack Muncher to idle
                    }

                    //Drive to backstage after x seconds
                    if (driveTimer.seconds() >= 1){
                        driveState = State.BACKBOARD_STACK2;
                        robot.followTrajectorySequenceAsync(BackDrop2);
                    }
                    break;
                case BACKBOARD_STACK2:

                    break;
            }

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

            //Updates lift PID control with current liftHeight variable
            lift.setHeight(liftHeight);

            poseEstimate = robot.getPoseEstimate();
            telemetry.addData("IMU Heading: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            //Updates heading every x seconds using the imu
            if(imuTimer.seconds() >= headingInterval) {
                //Keeps robots x and y, but assigns a new heading
                //robot.setPoseEstimate(new Pose2d(poseEstimate.getX(), poseEstimate.getY(), imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)+3.14159));
                imuTimer.reset();
            }
            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode;

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
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous
public class BlueLeftAsyncTruss extends LinearOpMode {

    //This enum defines steps of the trajectories
    enum State {
        BACKBOARD_DROP, //First drop of the yellow preload on the backboard
        PREDROP, //Then place the purple preload on the spikemark
        GENERAL_STACK, //Then drive to the white stacks
        INTAKE_STACK, //Wait for pixels to intake
        BACKBOARD_STACK, //Drive to backstage to place white pixels
        INTAKE_STACK2, //Drive to stack from the backstage
        BACKBOARD_STACK2, //Drive to backstage to place white pixels
        IDLE //Robot enters idle when finished
    }

    private IMU imu = null;
    Pose2d poseEstimate;

    //Instantiate our driveState
    State driveState = State.BACKBOARD_DROP;

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
    private int liftHeight = 0;
    private CRServo transfer = null;
    private Servo frontPincher, backPincher, wrist = null;
    private DcMotor leftLift,intake = null;

    //Timer for in between trajectories
    ElapsedTime driveTimer = new ElapsedTime();

    //Timer for periodically updating the heading with the imu
    ElapsedTime imuTimer = new ElapsedTime();
    private double headingInterval = 3;

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
    private BlueProcessor blueProcessor;

    @Override
    public void runOpMode() throws InterruptedException {
        //Instantiate SampleMecanumDrive
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        //Define robots starting position and orientation
        Pose2d startPose = new Pose2d(13, 65, Math.toRadians(180));
        robot.setPoseEstimate(startPose);

        TrajectorySequence BackBoardDropLeft = robot.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(52,42))
                //put pixel on backboard
                .addTemporalMarker(pathTime -> pathTime-1.5,() -> {
                    //Starts extending lift x seconds before reaching the backboard
                    liftState = LiftState.CLOSE_PINCHERS;
                    liftHeight = LiftConstants.liftAuto;
                })
                .addTemporalMarker(pathTime -> pathTime-0.2,() -> {
                    readyToDrop = true;
                })
                .build();
        TrajectorySequence PreDropLeft = robot.trajectorySequenceBuilder(BackBoardDropLeft.end())
                .lineTo(new Vector2d(33, 32))
                .build();
                //put pixel on left line
        TrajectorySequence WhiteStackOneLeft = robot.trajectorySequenceBuilder(PreDropLeft.end())
                .lineTo(new Vector2d(33, 50))
                .splineToConstantHeading(new Vector2d (10,57), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d (-30,57), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d (-35,57), Math.toRadians(180))
                .addTemporalMarker(pathTime -> pathTime-1.5,() -> {
                    DIservo.setPosition(LiftConstants.StackMuncher2);
                    intake.setPower(1);
                    transfer.setPower(1);
                })
                .lineToLinearHeading(new Pose2d(-60,44,  Math.toRadians(225)))
                .build();
        // pick up white pixels off stack


        TrajectorySequence BackBoardDropRight = robot.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(52,30))
                //place pixel on backboard
                .addTemporalMarker(pathTime -> pathTime-1.5,() -> {
                    //Starts extending lift
                    liftState = LiftState.CLOSE_PINCHERS;
                    liftHeight = LiftConstants.liftAuto;
                })
                .addTemporalMarker(pathTime -> pathTime-0.2,() -> {
                    readyToDrop = true;
                })
                .build();
        TrajectorySequence PreDropRight = robot.trajectorySequenceBuilder(BackBoardDropRight.end())
                .lineTo(new Vector2d(10, 35))
                .build();
        TrajectorySequence WhiteStackOneRight = robot.trajectorySequenceBuilder(PreDropRight.end())
                .lineTo(new Vector2d(15, 50))
                .splineToConstantHeading(new Vector2d (10,57), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d (-30,57), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d (-35,57), Math.toRadians(180))
                .addTemporalMarker(pathTime -> pathTime-1.5,() -> {
                    DIservo.setPosition(LiftConstants.StackMuncher2);
                    intake.setPower(1);
                    transfer.setPower(1);
                })
                .lineToLinearHeading(new Pose2d(-60,44,  Math.toRadians(225)))
        // pick up white pixels off stack
                .build();


        TrajectorySequence BackBoardDropMid = robot.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(52,36))
                // place pixel on backboard
                .addTemporalMarker(pathTime -> pathTime-1.5,() -> {
                    //Starts extending lift
                    liftState = LiftState.CLOSE_PINCHERS;
                    liftHeight = LiftConstants.liftAuto;
                })
                .addTemporalMarker(pathTime -> pathTime-0.2,() -> {
                    readyToDrop = true;
                })
                .build();
        TrajectorySequence PreDropMid = robot.trajectorySequenceBuilder(BackBoardDropMid.end())
                .lineTo(new Vector2d(20, 32))
                .build();
        TrajectorySequence WhiteStackOneMid = robot.trajectorySequenceBuilder(PreDropMid.end())
                .lineTo(new Vector2d(20, 50))
                .splineToConstantHeading(new Vector2d (10,57), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d (-30,57), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d (-35,57), Math.toRadians(180))
                .addTemporalMarker(pathTime -> pathTime-1.5,() -> {
                    DIservo.setPosition(LiftConstants.StackMuncher2);
                    intake.setPower(1);
                    transfer.setPower(1);
                })
                .lineToLinearHeading(new Pose2d(-56,38,  Math.toRadians(225)))
                .build();
        // pick up white pixels off stack

        //From stack to backstage beside the backdrop
        TrajectorySequence BackDrop = robot.trajectorySequenceBuilder(WhiteStackOneMid.end())
                .lineToLinearHeading(new Pose2d(-35,57,  Math.toRadians(-180)))
                .addTemporalMarker(0.5, () -> {
                    //Reject any extra pixel that might have been intaked
                    intake.setPower(-1);
                    transfer.setPower(1);
                })
                .addTemporalMarker(1.5,() -> {
                    //Turn off the intake
                    intake.setPower(0);
                    transfer.setPower(0);

                })
                .lineTo(new Vector2d(33,57))
                .splineToConstantHeading(new Vector2d(56,57), Math.toRadians(0))
                //place two white pixels
                //put pixel on backboard
                .addTemporalMarker(pathTime -> pathTime-1,() -> {
                    //Starts extending lift x seconds before reaching the backboard
                    liftState = LiftState.CLOSE_PINCHERS;
                    liftHeight = LiftConstants.liftAuto;
                })
                .addTemporalMarker(pathTime -> pathTime-0.2,() -> {
                    readyToDrop = true;
                })
                .build();

        //Moves from backstage to stack
        TrajectorySequence WhiteStackTwo = robot.trajectorySequenceBuilder(BackDrop.end())
                .lineTo(new Vector2d (-35,57))
                .lineToLinearHeading(new Pose2d(-60,44,  Math.toRadians(225)))
                .splineToConstantHeading(new Vector2d(50,32), Math.toRadians(0))
                //Turn on intake and lower the stack-muncher
                .addTemporalMarker(pathTime -> pathTime-1.5,() -> {
                    DIservo.setPosition(LiftConstants.StackMuncher2);
                    intake.setPower(1);
                    transfer.setPower(1);
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
        lift.init(hardwareMap);
        transfer = hardwareMap.get(CRServo.class, "transfer");
        preDropLeft = hardwareMap.get(Servo.class,  "preDropLeft");
        DIservo = hardwareMap.get(Servo.class, "DIservo");
        intake = hardwareMap.get(DcMotor.class, "intake");
        frontPincher = hardwareMap.get(Servo.class, "front_pincher");
        backPincher = hardwareMap.get(Servo.class, "back_pincher");
        wrist = hardwareMap.get(Servo.class, "wrist_servo");

        //init AprilTag & Colour processor
        aprilProcessor = new AprilTagProcessor.Builder()
                .build();
        blueProcessor = new BlueProcessor(telemetry);

        //init VisionPortal
        OpenCvVisionPortal = new VisionPortal.Builder()
                .setCamera(webcam1)
                .addProcessor(blueProcessor)
                .setCameraResolution(new Size(1920,1080))
                .enableLiveView(true)
                .build();

        //Updates telemetry with current prop location
        while (opModeInInit()){
            telemetry.addData("Location: ", blueProcessor.getLocation());
            telemetry.update();
        }
        waitForStart();

        //Sets beginning trajectory and drive state based off the blueProcessor's detection
        switch (blueProcessor.getLocation()) {
            case LEFT:
            case NOT_FOUND:
                robot.followTrajectorySequenceAsync(BackBoardDropLeft);
                break;
            case MIDDLE:
                robot.followTrajectorySequenceAsync(BackBoardDropMid);
                break;
            case RIGHT:
                robot.followTrajectorySequence(BackBoardDropRight);
                break;
        }

        imuTimer.reset();

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
            //State names describe what the robot is doing while in that state, not what the state contains
            switch (driveState) {
                case BACKBOARD_DROP:
                    //Wait for backboard trajectory to finish
                    if(!robot.isBusy()) {
                        //Runs the the different preDrop purple trajectories based on camera detection
                        switch (blueProcessor.getLocation()) {
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
                    if(!robot.isBusy()) {
                        //Drop preloaded purple pixel
                        preDropLeft.setPosition(0.85);

                        //Based on camera detection from beginning, run trajectory from spike mark to stacks
                        switch (blueProcessor.getLocation()) {
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
                        driveState = State.GENERAL_STACK;
                    }
                    break;
                case GENERAL_STACK:
                    if (!robot.isBusy()) {
                        //Reset timer once we've reached the stacks
                        driveTimer.reset();
                        driveState = State.INTAKE_STACK;
                    }
                    break;
                case INTAKE_STACK:
                    //Lower the Stack muncher for the second pixel after x seconds
                    if(driveTimer.seconds() >= 0.3)
                        DIservo.setPosition(LiftConstants.StackMuncher2);

                    //Drive to backstage after x seconds
                    if(driveTimer.seconds() >= 0.5) {
                        robot.followTrajectorySequenceAsync(BackDrop);
                        //Bring back the Stack Muncher to idle
                        DIservo.setPosition(LiftConstants.StackMuncherReturn);
                        driveState = State.BACKBOARD_STACK;
                    }
                    break;
                case BACKBOARD_STACK:
                    if (!robot.isBusy()) {
                        driveTimer.reset();
                        driveState = State.IDLE;
                    }
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
                    if (liftTimer.seconds() > 0.2){
                        backPincher.setPosition(backPincherClose);
                    }
                    //Wait for back pincher to close before extending lift
                    if (liftTimer.seconds() > 0.5) {
                        liftState = LiftState.LIFT_EXTEND;
                    }
                    break;
                case LIFT_EXTEND:
                    //Check if lift has fully extended
                    if (leftLift.getCurrentPosition() > 600) {
                        //Deploy box
                        lift.extendBox();
                        wrist.setPosition(LiftConstants.wristMiddle1);
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

            //stores the current robots position into a pose
            poseEstimate = robot.getPoseEstimate();
            telemetry.addData("IMU Heading: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            //Updates heading every x seconds using the imu
            if(imuTimer.seconds() >= headingInterval) {
                //Keeps robots x and y, but assigns a new heading
                robot.setPoseEstimate(new Pose2d(poseEstimate.getX(), poseEstimate.getY(), imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)+3.14159));
                imuTimer.reset();
            }
            telemetry.update();
        }
    }
}

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
public class BlueRightAsync extends LinearOpMode {

    //This enum defines steps of the trajectories
    enum State {
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
    private BlueProcessorFrontStage blueProcessorFrontStage;
    // private BlueProcessor blueProcessor;

    //Instantiate trajectories from stack to backstage seperately, because these need to be chained differently
    Trajectory TestBackDrop;
    TrajectorySequence TestBackDrop2;
    @Override
    public void runOpMode() throws InterruptedException {
        //Instantiate SampleMecanumDrive
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        //Define robots starting position and orientation
        Pose2d startPose = new Pose2d(-40, 62, Math.toRadians(90));
        robot.setPoseEstimate(startPose);

        TrajectorySequence BackBoardDropLeft = robot.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(52,44))
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

        TrajectorySequence PreDropLeft = robot.trajectorySequenceBuilder(BackBoardDropLeft.end())
                .lineTo(new Vector2d(33, 40))
                .addTemporalMarker(pathTime -> pathTime-0.2,() -> {
                    preDropRight.setPosition(0.85);
                })
                .build();
        //put pixel on left line

        TrajectorySequence WhiteStackOneLeft = robot.trajectorySequenceBuilder(PreDropLeft.end())
                .lineTo(new Vector2d(33, 51))
                .addTemporalMarker(0.5, () -> {
                    //Close the preDrop servo
                    preDropRight.setPosition(0.75);
                })
                .splineToConstantHeading(new Vector2d (10,59), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d (-15,58), Math.toRadians(180))
                .addTemporalMarker(pathTime -> pathTime - 3, () -> {
                    DIservo.setPosition(LiftConstants.StackMuncher1);
                })
                .addTemporalMarker(pathTime -> pathTime-2.5,() -> {
                    intake.setPower(1);
                    transfer.setPower(1);
                })
                .splineTo(new Vector2d(-47,38),Math.toRadians(200))
                .forward(5)
                .back(3)
                .addDisplacementMarker(() -> {
                    DIservo.setPosition(LiftConstants.StackMuncher2);
                    intake.setPower(-1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    intake.setPower(1);
                })
                .forward(3)
                .build();
        // pick up white pixels off stack


        TrajectorySequence PreDropRight = robot.trajectorySequenceBuilder(startPose)
                .back(2)
                .splineToConstantHeading(new Vector2d(-34, 25), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-53.5,17), Math.toRadians(180))
                //place pixel on right line
                .addTemporalMarker(pathTime -> pathTime-0.2,() -> {
                    preDropRight.setPosition(0.65);
                })
//                .addTemporalMarker(pathTime -> pathTime-1.5,() -> {
//                    //Starts extending lift
//                    liftState = LiftState.CLOSE_PINCHERS;
//                    storeLiftHeight = LiftConstants.liftAuto;
//                })
//                .addTemporalMarker(pathTime -> pathTime-0.2,() -> {
//                    readyToDrop = true;
//                })
                .build();

                TrajectorySequence WhiteStackOneRight = robot.trajectorySequenceBuilder(PreDropRight.end())
                .lineToLinearHeading(new Pose2d(-48,13, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-55, 13), Math.toRadians(180))
                        //pick up white pixel
                        .addTemporalMarker(0.5, () -> {
                            //Close the preDrop servo
                            preDropRight.setPosition(0.75);
                        })
                        .addTemporalMarker(pathTime -> pathTime - 2.8, () -> {
                            DIservo.setPosition(LiftConstants.StackMuncher1);
                        })
                        .addTemporalMarker(pathTime -> pathTime-2.5,() -> {
                            intake.setPower(1);
                            transfer.setPower(1);
                        })
                .build();

        TrajectorySequence BackBoardDropRight = robot.trajectorySequenceBuilder(WhiteStackOneRight.end())
                //.splineToConstantHeading(new Vector2d(-14,13), Math.toRadians(0))
                .addTemporalMarker(0.5, () -> {
                    intake.setPower(-1);
                })
                .addTemporalMarker(1.5,()->{
                    intake.setPower(0);
                    transfer.setPower(0);
                })
                .lineTo(new Vector2d(30,13))
                .splineToConstantHeading(new Vector2d(50,34),Math.toRadians(0))

                //place pixel on backboard
                .addTemporalMarker(pathTime -> pathTime -2, () -> {
                    liftState = LiftState.CLOSE_PINCHERS;
                    storeLiftHeight = LiftConstants.liftAuto;
                })
                .addTemporalMarker(pathTime -> pathTime -0.2, () -> {
                    readyToDrop = true;
        })
//                .splineTo(new Vector2d(-53,52.5),Math.toRadians(200))
//                .forward(3)
//                .back(3)
//                .addDisplacementMarker(() -> {
//                    DIservo.setPosition(LiftConstants.StackMuncher2);
//                    intake.setPower(1);
//
//                })
//                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {
//                    intake.setPower(-1);
//                })
//                .forward(3)
                .build();


        TrajectorySequence BackBoardDropMid = robot.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(52,38))
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
                .lineTo(new Vector2d(20, 30))
                .addTemporalMarker(pathTime -> pathTime-0.2,() -> {
                    preDropRight.setPosition(0.85);
                })
                .build();
        TrajectorySequence WhiteStackOneMid = robot.trajectorySequenceBuilder(PreDropMid.end())
                .lineTo(new Vector2d(20, 51))
                .addTemporalMarker(0.5, () -> {
                    //Close the preDrop servo
                    preDropRight.setPosition(0.75);
                })
                .splineToConstantHeading(new Vector2d (10,59), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d (-15,58), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d (-20,58), Math.toRadians(180))
                .addTemporalMarker(pathTime -> pathTime - 3, () -> {
                    DIservo.setPosition(LiftConstants.StackMuncher1);
                })
                .addTemporalMarker(pathTime -> pathTime-2.5,() -> {
                    intake.setPower(1);
                    transfer.setPower(1);
                })
                .splineTo(new Vector2d(-50,37.5),Math.toRadians(200))
                .forward(3)
                .back(3)
                .addDisplacementMarker(() -> {
                    DIservo.setPosition(LiftConstants.StackMuncher2);
                    intake.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {
                    intake.setPower(-1);
                })
                .forward(3)
                .build();

        // pick up white pixels off stack

        //From stack to backstage beside the backdrop
//        TestBackDrop = robot.trajectoryBuilder(WhiteStackOneMid.end(), true)
//                .addTemporalMarker(0.5, () -> {
//                    //Reject any extra pixel that might have been intaked
//                    intake.setPower(-1);
//                    transfer.setPower(1);
//                })
//                .splineTo(new Vector2d(-35, 57), Math.toRadians(-180))
//
//                .build();
//
//        TestBackDrop2 = robot.trajectorySequenceBuilder(TestBackDrop.end())
//                .addTemporalMarker(1.0,() -> {
//                    //Turn off the intake
//                    intake.setPower(0);
//                    transfer.setPower(0);
//
//                })
//                .splineTo(new Vector2d(33,57), Math.toRadians(180))
//                .splineTo(new Vector2d(56,57), Math.toRadians(180))
//                .addTemporalMarker(pathTime -> pathTime-2,() -> {
//                    //Starts extending lift x seconds before reaching the backboard
//                    liftState = LiftState.CLOSE_PINCHERS;
//                    storeLiftHeight = LiftConstants.liftAuto;
//                })
//                .addTemporalMarker(pathTime -> pathTime-0.2,() -> {
//                    readyToDrop = true;
//                })
//                .build();

        TrajectorySequence BackDrop = robot.trajectorySequenceBuilder(WhiteStackOneMid.end())
                .forward(2)
                .lineToLinearHeading(new Pose2d(-35,56,  Math.toRadians(-180)))
                .lineTo(new Vector2d(50,56))

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

        //Moves from backstage to stack
        TrajectorySequence WhiteStackTwo = robot.trajectorySequenceBuilder(BackDrop.end())
                .splineToConstantHeading(new Vector2d (10,53), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d (-20,53), Math.toRadians(180))
                .splineTo(new Vector2d(-49, 35), Math.toRadians(200))
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
                .lineToLinearHeading(new Pose2d(-35,52,  Math.toRadians(-180)))
                .lineTo(new Vector2d(50,52), SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 1.5))

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
        preDropRight = hardwareMap.get(Servo.class,  "preDropRight");
        DIservo = hardwareMap.get(Servo.class, "DIservo");
        intake = hardwareMap.get(DcMotor.class, "intake");
        frontPincher = hardwareMap.get(Servo.class, "front_pincher");
        backPincher = hardwareMap.get(Servo.class, "back_pincher");
        wrist = hardwareMap.get(Servo.class, "wrist_servo");
        boxBeam = hardwareMap.get(DigitalChannel.class, "box_beam");

        //init AprilTag & Colour processor
        aprilProcessor = new AprilTagProcessor.Builder()
                .build();
        blueProcessorFrontStage = new BlueProcessorFrontStage(telemetry);

       //   blueProcessor = new BlueProcessor(telemetry);

        //init VisionPortal
        OpenCvVisionPortal = new VisionPortal.Builder()
                .setCamera(webcam2)
                .addProcessor(blueProcessorFrontStage)
             //   .addProcessor(blueProcessor)

                .setCameraResolution(new Size(1920,1080))
                .enableLiveView(true)
                .build();

        //Updates telemetry with current prop location
        while (opModeInInit()){
            telemetry.addData("Location: ", blueProcessorFrontStage.getLocation());
            //   telemetry.addData("Location: ", blueProcessor.getLocation());
            telemetry.update();
        }
        waitForStart();

        //Sets beginning trajectory and drive state based off the blueProcessor's detection
       switch (blueProcessorFrontStage.getLocation()) {
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
            //State names describe what the robot is doing while in that state, not what the state contains
            switch (driveState) {
                case BACKBOARD_DROP:
                    //Wait for backboard trajectory to finish
                    if (!robot.isBusy()) {
                        //Runs the the different preDrop purple trajectories based on camera detection
                        switch (blueProcessorFrontStage.getLocation()) {
                     //           switch (blueProcessor.getLocation()) {
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
                        driveState = State.PREDROP;
                    }
                    break;
                case PREDROP:
                    if (!robot.isBusy()) {
                        //Drop preloaded purple pixel


                        //Based on camera detection from beginning, run trajectory from spike mark to stacks
                      switch (blueProcessorFrontStage.getLocation()) {
                       //     switch (blueProcessor.getLocation()) {
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
                        DIservo.setPosition(StackMuncherReturn);
                       // driveState = State.GENERAL_STACK;
                    }
                    break;
                case GENERAL_STACK:
                    if (!boxBeam.getState())
                        DIservo.setPosition(LiftConstants.StackMuncher2);

                    if (!robot.isBusy()) {
                        //Reset timer once we've reached the stacks
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
                        robot.followTrajectorySequenceAsync(BackDrop);
                    }
                    break;
                case BACKBOARD_STACK:
                    if (!robot.isBusy()) {
                        telemetry.addData("UH OH ", "AHHH");
                        robot.followTrajectorySequenceAsync(WhiteStackTwo);
                        driveTimer.reset();
                        driveState = State.GENERAL_STACK2;
                    }
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
            telemetry.addData("Lift State", liftState);
            //Updates lift PID control with current liftHeight variable
            lift.setHeight(liftHeight);

            //stores the current robots position into a pose
            poseEstimate = robot.getPoseEstimate();
            telemetry.addData("IMU Heading: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            //Updates heading every x seconds using the imu
            if(imuTimer.seconds() >= headingInterval) {
                //Keeps robots x and y, but assigns a new heading
//                robot.setPoseEstimate(new Pose2d(poseEstimate.getX(), poseEstimate.getY(), imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)+Math.toRadians(90)));
                imuTimer.reset();
            }
            telemetry.addData("Box Beam", boxBeam.getState());
            telemetry.update();
        }
    }
}


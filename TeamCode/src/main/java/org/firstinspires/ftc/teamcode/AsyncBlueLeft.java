package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.LiftConstants.liftRetracted;

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
public class AsyncBlueLeft extends LinearOpMode {

    //This enum defines steps of the trajectories
    enum State {
        BACKBOARD_DROP, //First drop of the yellow preload on the backboard
        BACKBOARD_WAIT1, // Wait for pixels to drop
        PREDROP, //Then place the purple preload on the spikemark
        DROP_WAIT, //Short wait to drop purple pixel
        GENERAL_STACK, //Then drive to in front of the white stacks
        INTAKE_STACK, //Drive into stack with intake on
        INTAKE_WAIT, //Wait for pixels to intake
        BACKBOARD_STACK, //Drive to backboard to place white pixels
        BACKBOARD_WAIT2, // Wait for pixels to drop
        PARK, //Park in backstage
        IDLE //Robot enters idle when finished
    }

    private IMU imu = null;
    Pose2d poseEstimate;

    //Instantiate our driveState
    State driveState = State.IDLE;

    //Declare purple preload servo
    private Servo preDropLeft = null;

    //Declare stack muncher servo
    private Servo DIservo;

    //Instantiate our lift class
    PidControl2 lift =new PidControl2();

    //This enum defines steps of the lift
    private enum LiftState {
        LIFT_IDLE,
        LIFT_EXTEND,
        BOX_EXTEND,
        LIFT_DUMP,
        BOX_RETRACT,
        LIFT_RETRACT,
    }
    ElapsedTime liftTimer = new ElapsedTime();
    LiftState liftState = LiftState.LIFT_EXTEND;
    private int liftHeight = 0;
    private CRServo IOservo = null;
    private DcMotor leftLift,intake = null;

    //Timer for in between trajectories
    ElapsedTime driveTimer = new ElapsedTime();

    //Timer for periodically updating the heading with the imu
    ElapsedTime imuTimer = new ElapsedTime();
    private double headingInterval = 3;

    //Lets the lift state machine know when the trajectory to the backboard is finished
    private boolean readyToDrop = false;

    //Time for pixels to drop from bucket
    private double dropTime = 2;

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
                .addTemporalMarker(pathTime -> pathTime-1,() -> {
                    //Starts extending lift x seconds before reaching the backboard
                    liftState = LiftState.LIFT_EXTEND;
                    liftHeight = LiftConstants.liftAuto;
                })
                .addTemporalMarker(pathTime -> pathTime-0.5,() -> {
                    readyToDrop = true;
                })
                .build();
        TrajectorySequence PreDropLeft = robot.trajectorySequenceBuilder(BackBoardDropLeft.end())
                .lineTo(new Vector2d(33, 32))
                .build();

        TrajectorySequence BackBoardDropRight = robot.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(52,30))
                //place pixel on backboard
                .addTemporalMarker(pathTime -> pathTime-1,() -> {
                    //Starts extending lift
                    liftState = LiftState.LIFT_EXTEND;
                    liftHeight = LiftConstants.liftAuto;
                })
                .addTemporalMarker(pathTime -> pathTime-0.5,() -> {
                    readyToDrop = true;
                })
                .build();
        TrajectorySequence PreDropRight = robot.trajectorySequenceBuilder(BackBoardDropRight.end())
                .lineTo(new Vector2d(10, 35))
                .build();

        TrajectorySequence BackBoardDropMid = robot.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(52,36))
                .addTemporalMarker(pathTime -> pathTime-1,() -> {
                    //Starts extending lift
                    liftState = LiftState.LIFT_EXTEND;
                    liftHeight = LiftConstants.liftAuto;
                })
                .addTemporalMarker(pathTime -> pathTime-0.5,() -> {
                    readyToDrop = true;
                })
                .build();
        TrajectorySequence PreDropMid = robot.trajectorySequenceBuilder(BackBoardDropMid.end())
                .lineTo(new Vector2d(20, 31.5))
                .build();

        //Moves to position in middle before going to stacks
        TrajectorySequence Generalposition = robot.trajectorySequenceBuilder(PreDropMid.end())
                .lineTo(new Vector2d(33, 8))
                .build();
        //Moves to white pixel stack on left line
        TrajectorySequence WhiteStackPath = robot.trajectorySequenceBuilder(Generalposition.end())
                .lineTo(new Vector2d (-52, 8))
                .addTemporalMarker(pathTime -> pathTime-1.5,() -> {
                    DIservo.setPosition(LiftConstants.StackMuncher2);
                    intake.setPower(1);
                })
                .build();
        //Moves from stack to the backdrop
        TrajectorySequence StacktoBackBoardMid = robot.trajectorySequenceBuilder(WhiteStackPath.end())
                .strafeTo(new Vector2d(38, 8))
                .addTemporalMarker(0.5, () -> {
                    //Reject any extra pixel that might have been intaked
                    intake.setPower(-1);
                })
                .addTemporalMarker(1.5,() -> {
                    //Turn off the intake
                    intake.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(50,32), Math.toRadians(0))
                .addTemporalMarker(pathTime -> pathTime-1,() -> {
                    //Starts extending lift
                    liftState = LiftState.LIFT_EXTEND;
                    liftHeight = LiftConstants.liftAuto;
                })
                .addTemporalMarker(pathTime -> pathTime-0.5,() -> {
                    readyToDrop = true;
                })
                .build();
        //Parks the robot on the outside of the backstage
        TrajectorySequence Park = robot.trajectorySequenceBuilder(StacktoBackBoardMid.end())
                .splineToConstantHeading(new Vector2d(56,60), Math.toRadians(0))
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
        IOservo = hardwareMap.get(CRServo.class, "IOservo");
        preDropLeft = hardwareMap.get(Servo.class,  "preDropLeft");
        DIservo = hardwareMap.get(Servo.class, "DIservo");
        intake = hardwareMap.get(DcMotor.class, "intake");

        //initAuto AprilTag & Colour processor
        aprilProcessor = new AprilTagProcessor.Builder()
                .build();
        blueProcessor = new BlueProcessor(telemetry);

        //initAuto VisionPortal
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
        driveState = State.BACKBOARD_DROP;

        imuTimer.reset();
        imu.resetYaw();

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
                    if (!robot.isBusy()) {
                        driveTimer.reset();
                        driveState = State.BACKBOARD_WAIT1;
                    }
                    break;
                case BACKBOARD_WAIT1:
                    //Wait for x seconds required for pixels to fall out
                    if(driveTimer.seconds() >= dropTime) {
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
                        driveState = State.DROP_WAIT;
                        driveTimer.reset();
                        preDropLeft.setPosition(0.1);
                    }
                    break;
                case DROP_WAIT:
                    if(driveTimer.seconds() >= 0.4) {
                        robot.followTrajectorySequenceAsync(Generalposition);
                        driveState = State.GENERAL_STACK;
                    }
                    break;
                case GENERAL_STACK:
                    if(!robot.isBusy()) {
                        robot.followTrajectorySequenceAsync(WhiteStackPath);
                        driveState = State.INTAKE_STACK;
                    }
                    break;
                case INTAKE_STACK:
                    if(!robot.isBusy()) {
                        driveTimer.reset();
                        driveState = State.INTAKE_WAIT;
                    }
                    break;
                case INTAKE_WAIT:
                    //Lower the Stack muncher for the second pixel after x seconds
                    if(driveTimer.seconds() >= 0.5)
                        DIservo.setPosition(LiftConstants.StackMuncher2);

                    if(driveTimer.seconds() >= 1) {
                        robot.followTrajectorySequenceAsync(StacktoBackBoardMid);
                        //Bring back the Stack Muncher to idle
                        DIservo.setPosition(LiftConstants.StackMuncherReturn);
                        driveState = State.BACKBOARD_STACK;
                    }
                    break;
                case BACKBOARD_STACK:
                    if (!robot.isBusy()) {
                        driveTimer.reset();
                        driveState = State.BACKBOARD_WAIT2;
                    }
                    break;
                case BACKBOARD_WAIT2:
                    if (driveTimer.seconds() >= dropTime) {
                        robot.followTrajectorySequenceAsync(Park);
                        driveState = State.PARK;
                    }
                    break;
                case PARK:
                    break;
            }

            //State machine for controlling lift
            switch (liftState) {
                case LIFT_IDLE:
                    break;
                case LIFT_EXTEND:
                    //Check if lift has fully extended
                    if (Math.abs(leftLift.getCurrentPosition() - liftHeight) < 20) {
                        //Deploy box
                        lift.extendBox();
                        liftState = LiftState.BOX_EXTEND;
                        liftTimer.reset();
                    }
                    break;
                case BOX_EXTEND:
                    //Wait for servo to reach position
                    if (readyToDrop) {
                        //Spin out pixels and move to next state
                        liftState = LiftState.LIFT_DUMP;
                        IOservo.setPower(-1);
                        liftTimer.reset();
                    }
                    break;
                case LIFT_DUMP:
                    //Wait 2 seconds for pixels to spin out
                    if(liftTimer.seconds() >= dropTime) {
                        //Turn off IOservo and retract box
                        IOservo.setPower(0);
                        lift.retractBox();
                        liftTimer.reset();
                        liftState = LiftState.BOX_RETRACT;
                    }
                    break;
                case BOX_RETRACT:
                    // Wait for servo to return to Idle
                    if (liftTimer.seconds() >= 0.3) {
                        liftState = LiftState.LIFT_RETRACT;
                        //Retract Lift
                        liftHeight = liftRetracted;
                    }
                    break;
                case LIFT_RETRACT:
                    //Wait for Lift to return to idle
                    if (Math.abs(leftLift.getCurrentPosition() - liftRetracted) < 10) {
                        liftState = LiftState.LIFT_IDLE;
                    }
                    break;
                default:
                    //Should never happen but just in case
                    liftState = LiftState.LIFT_IDLE;
            }

            //Updates lift PID control with current liftHeight variable
            lift.setHeight(liftHeight);

            Pose2d poseEstimate = robot.getPoseEstimate();
            telemetry.addData("IMU: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            telemetry.addData("Heading: ", poseEstimate.getHeading());
            //Updates heading every x seconds using the imu
            if(imuTimer.seconds() >= headingInterval) {
                robot.setPoseEstimate(new Pose2d(poseEstimate.getX(), poseEstimate.getY(), imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)+3.14159));
                imuTimer.reset();
            }
            telemetry.update();
        }
    }
}

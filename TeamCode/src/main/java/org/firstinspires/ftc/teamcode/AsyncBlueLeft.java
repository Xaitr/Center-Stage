package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.LiftConstants.liftRetracted;

import android.graphics.Canvas;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

import java.util.ArrayList;

@Autonomous
public class AsyncBlueLeft extends LinearOpMode {

    //This enum defines steps of the trajectories
    enum State {
        BACKBOARD_DROP, //First drop of the yellow preload on the backboard
        PREDROP, //Then place the purple preload on the spikemark
        DROP_WAIT, //Short wait to drop purple pixel
        GENERAL_STACK, //Then drive to in front of the white stacks
        INTAKE_STACK, //Drive into stack with intake on
        BACKBOARD_STACK, //Drive to backboard to place white pixels
        PARK, //Park in backstage
        IDLE //Robot enters idle when finished
    }

    //Instantiate our driveState
    State driveState = State.IDLE;

    //Declare purple preload servo
    private Servo preDropLeft = null;

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
    private DcMotor leftLift = null;

    //Timer for in between trajectories
    ElapsedTime driveTimer = new ElapsedTime();

    //Lets the lift state machine know when the trajectory to the backboard is finished
    private boolean readyToDrop = false;

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
                .build();
        TrajectorySequence PreDropLeft = robot.trajectorySequenceBuilder(BackBoardDropLeft.end())
                .lineTo(new Vector2d(33, 32))
                .build();

        TrajectorySequence BackBoardDropRight = robot.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(52,30))
                //place pixel on backboard
                .build();
        TrajectorySequence PreDropRight = robot.trajectorySequenceBuilder(BackBoardDropRight.end())
                .lineTo(new Vector2d(10, 35))
                .build();

        TrajectorySequence BackBoardDropMid = robot.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(52,36))
                .build();
        TrajectorySequence PreDropMid = robot.trajectorySequenceBuilder(BackBoardDropMid.end())
                .lineTo(new Vector2d(20, 31.5))
                .build();

        //Moves to position in middle before going to stacks
        TrajectorySequence Generalposition = robot.trajectorySequenceBuilder(PreDropMid.end())
                .lineTo(new Vector2d(33, 8))
                .build();
        //Moves to white pixel stack on left line
        TrajectorySequence WhiteStackPathMid = robot.trajectorySequenceBuilder(Generalposition.end())
                .lineTo(new Vector2d (-56, 8))
                .build();
        //Moves from stack to the backdrop
        TrajectorySequence StacktoBackBoardMid = robot.trajectorySequenceBuilder(WhiteStackPathMid.end())
                .strafeTo(new Vector2d(38, 8))
                .splineToConstantHeading(new Vector2d(50,32), Math.toRadians(0))
                .build();
        //Parks the robot on the outside of the backstage
        TrajectorySequence ParkMid = robot.trajectorySequenceBuilder(StacktoBackBoardMid.end())
                .splineToConstantHeading(new Vector2d(56,60), Math.toRadians(0))
                .build();

        //Assign both webcams
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");

        //init AprilTag & Colour processor
        aprilProcessor = new AprilTagProcessor.Builder()
                .build();
        blueProcessor = new BlueProcessor();

        //init VisionPortal
        OpenCvVisionPortal = new VisionPortal.Builder()
                .setCamera(webcam1)
                .setCamera(webcam2)
                .addProcessor(blueProcessor)
                .addProcessor(aprilProcessor)
                .build();

        //Enable colour processor and webcam1
        OpenCvVisionPortal.setProcessorEnabled(blueProcessor, true);
        OpenCvVisionPortal.setActiveCamera(webcam1);

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

        //Start raising the lift right at the beginning.
        liftHeight = LiftConstants.liftAuto;

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
                        driveState = State.DROP_WAIT;
                        driveTimer.reset();
                        preDropLeft.setPosition(0.85);
                    }
                    break;
                case DROP_WAIT:
                    if(driveTimer.seconds() >= 0.4) {
                        robot.followTrajectorySequenceAsync(Generalposition);
                        driveState = State.GENERAL_STACK;
                    }
                    break;
                case GENERAL_STACK:

            }

            //State machine for controlling lift
            switch (liftState) {
                case LIFT_IDLE:
                    //Move to next state when lift height is changed through Roadrunner markers
                    if (liftHeight > liftRetracted) {
                        liftState = LiftState.LIFT_EXTEND;
                    }
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
                    if (liftTimer.seconds() >= 0.3) {
                        //Spin out pixels and move to next state
                        liftState = LiftState.LIFT_DUMP;
                        IOservo.setPower(-1);
                        liftTimer.reset();
                    }
                    break;
                case LIFT_DUMP:
                    //Wait 2 seconds for pixels to spin out
                    if(liftTimer.seconds()>=2) {
                        //Turn off IOservo and retract box
                        IOservo.setPower(0);
                        lift.retractBox();
                        liftTimer.reset();
                        liftState = LiftState.LIFT_RETRACT;
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
        }
    }
}

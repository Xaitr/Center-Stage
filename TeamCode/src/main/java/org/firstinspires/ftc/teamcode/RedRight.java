package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
public class RedRight extends LinearOpMode {
    Outake outake = new Outake();
    Intake intake = new Intake();
    PidControl2 lift = new PidControl2();
    private Servo preDropRight = null;
    OpenCvCamera camera;
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





    @Override
    public void runOpMode() throws InterruptedException {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        OpenCv detector = new OpenCv (telemetry);
        camera.setPipeline(detector);
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(13, -65, Math.toRadians(180));
        robot.setPoseEstimate(startPose);
        Pose2d prePark = new Pose2d(0, 0, 0);
        intake.init(hardwareMap);
        leftLift = hardwareMap.get(DcMotorEx.class, "left_lift");
        IOservo = hardwareMap.get(CRServo.class, "IOservo");
        rightServo = hardwareMap.get(Servo.class, "Right_outtake");
        preDropRight  = hardwareMap.get(Servo.class,  "preDropRight");
        lift.initAuto(hardwareMap);


        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        //TrajectorySequence Right = robot.trajectorySequenceBuilder(startPose)
        TrajectorySequence BackBoardDropRight = robot.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(53,-44))
                .build();
        //put pixel on backboard
        TrajectorySequence PreDropRight = robot.trajectorySequenceBuilder(BackBoardDropRight.end())
                .lineTo(new Vector2d(33, -33))
                .build();
        //put pixel on left line

        TrajectorySequence WhiteStackPathRight = robot.trajectorySequenceBuilder(PreDropRight.end())
                .lineTo(new Vector2d(33, -8))
                .lineTo(new Vector2d (-56, -8))
                .build();
        //intake pixel off white stack
        TrajectorySequence StacktoBackBoardRight = robot.trajectorySequenceBuilder(WhiteStackPathRight.end())
                .strafeTo(new Vector2d(38, -8))
                .splineToConstantHeading(new Vector2d(50,-32), Math.toRadians(0))
                .build();
        // put white pixels on backboardboard
        TrajectorySequence ParkRight = robot.trajectorySequenceBuilder(StacktoBackBoardRight.end())
                .splineToConstantHeading(new Vector2d(56,-60), Math.toRadians(0))
                .build();
        //park


        //TrajectorySequence Left = robot.trajectorySequenceBuilder(startPose)
        TrajectorySequence BackBoardDropLeft = robot.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(53,-30))
                //place pixel on backboard
                .build();
        TrajectorySequence PreDropLeft = robot.trajectorySequenceBuilder(BackBoardDropLeft.end())
                .lineTo(new Vector2d(10.5, -35))
                .build();
        //place pixel on line
        TrajectorySequence WhiteStackPathLeft = robot.trajectorySequenceBuilder(PreDropLeft.end())
                .lineToConstantHeading (new Vector2d(15, -32))
                .splineToConstantHeading(new Vector2d(10,-8), Math.toRadians(90))
                .lineTo(new Vector2d (-56, -8))
                .build();
        //intake pixel off white stack
        TrajectorySequence StacktoBackBoardLeft = robot.trajectorySequenceBuilder(WhiteStackPathLeft.end())
                .strafeTo(new Vector2d(16,-8))
                .splineToConstantHeading(new Vector2d(50,-32), Math.toRadians(0))
                .build();
        // put white pixels on backboardboard
        TrajectorySequence ParkLeft = robot.trajectorySequenceBuilder(StacktoBackBoardLeft.end())
                .splineToConstantHeading(new Vector2d(46,-60), Math.toRadians(0))
                .build();
        //park

        //TrajectorySequence Middle = robot.trajectorySequenceBuilder(startPose)
        TrajectorySequence BackBoardDropMid = robot.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(53,-36))
                .build();
        //place pixel on backboard
        TrajectorySequence PreDropMid = robot.trajectorySequenceBuilder(BackBoardDropMid.end())
                .strafeRight(10)
                .lineTo(new Vector2d(15, -22))
                .back(4)
                .build();
        //place pixel on line
        TrajectorySequence WhiteStackPathMid = robot.trajectorySequenceBuilder(PreDropMid.end())
                .lineTo(new Vector2d(24, -8))
                .lineTo(new Vector2d (-56, -8))
                .build();
        //intake pixel off white stack
        TrajectorySequence StacktoBackBoardMid = robot.trajectorySequenceBuilder(WhiteStackPathMid.end())
                .strafeTo(new Vector2d(16,-8))
                .strafeTo(new Vector2d(38, -8))
                .splineToConstantHeading(new Vector2d(50,-32), Math.toRadians(0))
                .build();
        // put white pixels on backboardboard
        TrajectorySequence ParkMid = robot.trajectorySequenceBuilder(StacktoBackBoardMid.end())
                .splineToConstantHeading(new Vector2d(56,-60), Math.toRadians(0))
                .build();
        //park

        waitForStart();


        switch (detector.getLocation()) {
            case LEFT:
                telemetry.addData("Left side", "proceed"); // open cv detects left spike
                telemetry.update();
                robot.followTrajectorySequence(BackBoardDropLeft);
                while(liftState != LiftState.LIFT_DONE){
                    switch (liftState) {
                        case LIFT_EXTEND:
                            //Extend lift
                            liftHeight = LiftConstants.liftAuto;
                            //Check if lift has fully extended
                            if (Math.abs(leftLift.getCurrentPosition() - liftHeight) < 15) {
                                //Deploy box
                                liftState = LiftState.BOX_EXTEND;
                                lift.extendBox();
                            }
                            break;
                        case BOX_EXTEND:
                            //Wait for servo to reach position
                            if (rightServo.getPosition() == LiftConstants.BoxReady) {
                                liftState = LiftState.LIFT_DUMP;
                            }
                            break;
                        case LIFT_DUMP:
                            liftState = LiftState.BOX_RETRACT;
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
                preDropRight.setPosition(0.65);
                sleep(1000);
                preDropRight.setPosition(0.75);
                robot.followTrajectorySequence(ParkLeft);
                break;


            case RIGHT:
                telemetry.addData("Right Side", "proceed");
                telemetry.update();
                robot.followTrajectorySequence(BackBoardDropRight);
                while(liftState != LiftState.LIFT_DONE){
                    switch (liftState) {
                        case LIFT_EXTEND:
                            //Extend lift
                            liftHeight = LiftConstants.liftAuto;
                            //Check if lift has fully extended
                            if (Math.abs(leftLift.getCurrentPosition() - liftHeight) < 15) {
                                //Deploy box
                                liftState = LiftState.BOX_EXTEND;
                                lift.AutoBoxReady();
                            }
                            break;
                        case BOX_EXTEND:
                            //Wait for servo to reach position
                            if (rightServo.getPosition() == LiftConstants.AutoBoxReady) {
                                liftState = LiftState.LIFT_DUMP;
                            }
                            break;
                        case LIFT_DUMP:
                            liftState = LiftState.BOX_RETRACT;
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

                robot.followTrajectorySequence(PreDropRight);
                preDropRight.setPosition(0.65);
                sleep(1000);
                preDropRight.setPosition(0.75);
                robot.followTrajectorySequence(ParkRight);
                break;



            case MIDDLE:
                telemetry.addData("Middle", "proceed");
                telemetry.update();
                robot.followTrajectorySequence(BackBoardDropMid);
                while(liftState != LiftState.LIFT_DONE){
                    switch (liftState) {
                        case LIFT_EXTEND:
                            //Extend lift
                            liftHeight = LiftConstants.liftAuto;
                            //Check if lift has fully extended
                            if (Math.abs(leftLift.getCurrentPosition() - liftHeight) < 15) {
                                //Deploy box
                                liftState = LiftState.BOX_EXTEND;
                                lift.AutoBoxReady();
                            }
                            break;
                        case BOX_EXTEND:
                            //Wait for servo to reach position
                            if (rightServo.getPosition() == LiftConstants.AutoBoxReady) {
                                liftState = LiftState.LIFT_DUMP;
                            }
                            break;
                        case LIFT_DUMP:
                            liftState = LiftState.BOX_RETRACT;
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
                robot.followTrajectorySequence(PreDropMid);
                preDropRight.setPosition(0.65);
                sleep(1000);
                preDropRight.setPosition(0.75);
                robot.followTrajectorySequence(ParkMid);
                break;



            case NOT_FOUND:
                telemetry.addData("not found", "proceed");
                telemetry.update();
                robot.followTrajectorySequence(BackBoardDropRight);
                robot.followTrajectorySequence(ParkLeft);
        }
        camera.stopStreaming();
    }
}
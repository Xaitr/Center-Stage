package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class BlueLeft extends LinearOpMode {
    Outake outake = new Outake();
    Intake intake = new Intake();
    PidControl2 lift =new PidControl2();
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


    @Override
    public void runOpMode() throws InterruptedException {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        OpenCvblue detector = new OpenCvblue (telemetry);
        camera.setPipeline(detector);
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(13, 65, Math.toRadians(180));
        robot.setPoseEstimate(startPose);
        Pose2d prePark = new Pose2d(0,0,0);
        intake.init(hardwareMap);
        leftLift = hardwareMap.get(DcMotorEx.class, "left_lift");
        IOservo = hardwareMap.get(CRServo.class, "IOservo");
        rightServo = hardwareMap.get(Servo.class, "Right_outtake");
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
        TrajectorySequence Left = robot.trajectorySequenceBuilder(startPose)
                .strafeLeft(1)
                .back(6)
                .splineToConstantHeading(new Vector2d(36,33), Math.toRadians(0))
                // spit out pixel here
                .addTemporalMarker(() -> {
                    intake.Reject();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    intake.RejectOff();
                })
                .strafeTo(new Vector2d (52, 44))
                // put pixel on board
                .build();
        TrajectorySequence Right = robot.trajectorySequenceBuilder(startPose)
                .strafeLeft(1)
                .back(0.5)
            .splineToConstantHeading(new Vector2d(15.5,32), Math.toRadians(0))
                 .addTemporalMarker(() -> {
                     intake.Reject();
                 })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    intake.RejectOff();
                })
            // spit out pixel here
            .strafeTo(new Vector2d (54, 28))
            // put pixel on board

            .build();
        TrajectorySequence Park = robot.trajectorySequenceBuilder(new Pose2d(51, 37, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(48,37))
                .splineToConstantHeading(new Vector2d(56,63), Math.toRadians(0))
                .build();
        TrajectorySequence Middle = robot.trajectorySequenceBuilder(startPose)
                .strafeLeft(1)
                .back(5)
            .splineToConstantHeading(new Vector2d(28, 24.5), Math.toRadians(0))
             .addTemporalMarker(() -> {
                intake.Reject();
            })
            .waitSeconds(0.5)
            .addTemporalMarker(() -> {
                intake.RejectOff();
            })
             .lineToConstantHeading(new Vector2d(53.5, 38))
        // put pixel on board
                .build();
        waitForStart();


        switch (detector.getLocation()) {
            case LEFT:
                telemetry.addData("Left side","proceed"); // open cv detects left spike
                telemetry.update();
                 robot.followTrajectorySequence(Left);
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
               robot.followTrajectorySequence(Park);
               break;

            case RIGHT:
                telemetry.addData("Right Side","proceed");
                telemetry.update();
                robot.followTrajectorySequence(Right);
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
                robot.followTrajectorySequence(Park);
        break;


            case MIDDLE:
                telemetry.addData("Middle","proceed");
                telemetry.update();
                robot.followTrajectorySequence(Middle);
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
               robot.followTrajectorySequence(Park);
                break;



            case NOT_FOUND:
                telemetry.addData("not found","proceed");
                telemetry.update();
                robot.followTrajectorySequence(Left);
        }
        camera.stopStreaming();
    }
}
package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvSwitchableWebcam;

@Autonomous
public class BlueRight extends LinearOpMode {
    Outake outake = new Outake();
    Driving driving = new Driving();
    OpenCvCamera camera;
    Intake intake = new Intake();
    private Servo preDropLeft = null;

    PidControl2 lift =new PidControl2();

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
        OpenCvblue detector = new OpenCvblue(telemetry);
        camera.setPipeline(detector);
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-37, 65, Math.toRadians(180));
        robot.setPoseEstimate(startPose);
        intake.init(hardwareMap);
        preDropLeft = hardwareMap.get(Servo.class,  "preDropLeft");
        leftLift = hardwareMap.get(DcMotorEx.class, "left_lift");
        IOservo = hardwareMap.get(CRServo.class, "IOservo");
        rightServo = hardwareMap.get(Servo.class, "Right_outtake");
        lift.init(hardwareMap);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
        //TrajectorySequence Left = robot.trajectorySequenceBuilder(startPose)
        TrajectorySequence PreDropLeft = robot.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-36, 30))
                .build();
                // spit out pixel on line
        TrajectorySequence BackBoardLeft = robot.trajectorySequenceBuilder(PreDropLeft.end())
                .lineTo(new Vector2d(-34,18))
                .splineToConstantHeading(new Vector2d(-14, 8), Math.toRadians(0))
                .lineTo(new Vector2d(30,8))
                .splineToConstantHeading(new Vector2d(50,32), Math.toRadians(0))
                .build();
                //place pixel on backboard
        TrajectorySequence ParkLeft = robot.trajectorySequenceBuilder(BackBoardLeft.end())
                .splineToConstantHeading(new Vector2d(56,8), Math.toRadians(0))
                .build();




       // TrajectorySequence Right = robot.trajectorySequenceBuilder(startPose)
        TrajectorySequence PreDropRight = robot.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-36, 30))
                // spit out pixel here
                .build();
        TrajectorySequence BackBoardRight = robot.trajectorySequenceBuilder(PreDropRight.end())
                .lineTo(new Vector2d(-34,18))
                .splineToConstantHeading(new Vector2d(-14, 8), Math.toRadians(0))
                .lineTo(new Vector2d(30,8))
                .splineToConstantHeading(new Vector2d(50,32), Math.toRadians(0))
                .build();
        TrajectorySequence ParkRight = robot.trajectorySequenceBuilder(BackBoardRight.end())
                .splineToConstantHeading(new Vector2d(56,8), Math.toRadians(0))
                .build();



       //TrajectorySequence Middle = robot.trajectorySequenceBuilder(startPose)
        TrajectorySequence PreDropMid = robot.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-48,47))
                .strafeLeft(30)
                .lineToLinearHeading(new Pose2d(-36, 15, Math.toRadians(75)))
                //spit out pixel here
                .build();
         TrajectorySequence BackBoardMid = robot.trajectorySequenceBuilder(PreDropMid.end())
                 .lineTo(new Vector2d(-36,13))
                 .turn(Math.toRadians(105))
                 .lineTo(new Vector2d(20,13),
                         SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                         SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                 .splineToConstantHeading(new Vector2d(50.5,35), Math.toRadians(0),
                         SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                         SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //place pixel on backboard
                 .build();
                 TrajectorySequence ParkMid = robot.trajectorySequenceBuilder(BackBoardMid.end())
                         .splineToConstantHeading(new Vector2d(48,8), Math.toRadians(0))
                //park
                .build();

        waitForStart();
        // camera.setActiveCamera(webcam1);


        switch (detector.getLocation()) {
            case LEFT:
                telemetry.addData("Left side", "proceed"); // open cv detects left spike
                telemetry.update();
                robot.followTrajectorySequence(PreDropLeft);


                preDropLeft.setPosition(0.85);
                sleep(1000);
                robot.followTrajectorySequence(BackBoardLeft);
                while(liftState != LiftState.LIFT_DONE){
                    switch (liftState) {
                        case LIFT_EXTEND:
                            //Extend lift
                            liftHeight = 1250;
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
                robot.followTrajectorySequence(ParkLeft);
                break;


            case RIGHT:
                telemetry.addData("Right Side", "proceed");
                telemetry.update();
                robot.followTrajectorySequence(PreDropRight);
                preDropLeft.setPosition(0.85);
                sleep(1000);
                robot.followTrajectorySequence(BackBoardRight);
                robot.followTrajectorySequence(ParkRight);
                break;


            case MIDDLE:
                telemetry.addData("Middle", "proceed");
                telemetry.update();
                robot.followTrajectorySequence(PreDropMid);
                preDropLeft.setPosition(0.85);
                sleep(1000);
                robot.followTrajectorySequence(BackBoardMid);
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
                robot.followTrajectorySequence(ParkMid);
                break;



            case NOT_FOUND:
                telemetry.addData("not found", "proceed");
                telemetry.update();

        }
        camera.stopStreaming();
    }
}

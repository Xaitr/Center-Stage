package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class Driving extends OpMode
{
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftLift = null;

    private CRServo IOservo = null; // intake and outtake servo
    private Servo rightServo = null;
    private Servo leftServo =null;


    private DcMotor Intake = null;

    PidControl2 lift = new PidControl2();
    // Enum to represent lift state
    private enum LiftState {
        LIFT_START,
        LIFT_EXTEND,
        BOX_EXTEND,
        LIFT_DUMP,
        BOX_RETRACT,
        LIFT_RETRACT,
        LIFT_RETRACTED
    }

    //Timer for waiting for pixels to spin out
    ElapsedTime liftTimer = new ElapsedTime();

    //Initial Lift Position
    LiftState liftState = LiftState.LIFT_START;

    @Override
    public void init() {

        lift.init(hardwareMap);
        //Declare variables for phone to recognise//

        //names on the config

        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");
        leftLift = hardwareMap.get(DcMotorEx.class, "left_lift");

        IOservo = hardwareMap.get(CRServo.class, "IOservo");
        Intake=hardwareMap.get(DcMotor.class, "intake");
        rightServo = hardwareMap.get(Servo.class, "Right_outtake");
        leftServo = hardwareMap.get(Servo.class, "Left_outtake");




        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightServo.setDirection(Servo.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("status", "Initialized");
    }


    //Set variables//
    @Override
    public void loop() {
        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;


        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        leftFrontPower = Range.clip(drive + turn - strafe, -1, 1);
        rightFrontPower = Range.clip(drive - turn + strafe, -1, 1);
        leftBackPower = Range.clip(drive + turn + strafe, -1, 1);
        rightBackPower = Range.clip(drive - turn - strafe, -1, 1);

        if(gamepad1.right_bumper){
            leftFrontPower /= 2;
            leftBackPower /= 2;
            rightFrontPower /= 2;
            rightBackPower /= 2;
        }
       if (gamepad2.a) {
           Intake.setPower(1);
           IOservo.setPower(1);
       }
       else if (gamepad2.b) {
            Intake.setPower(-1);
            IOservo.setPower(1);


    }
       else {
       Intake.setPower(0);
       IOservo.setPower(0);
       }
/*
       if (gamepad2.x) {
           outake.setHeight(100);
//           IOservo.setPower(-1);
       }
       if (gamepad2.right_bumper) {
           RightServo.setPosition(0);
           LeftServo.setPosition(0);
       }
       else if (gamepad2.left_bumper) {
           RightServo.setPosition(0.25);
           LeftServo.setPosition(0.25);
       }
*/

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        //Claw Code: Opens with GP2 X and opens less when past vertical position
        // BIGGER CLOSES MORE*********************

        //Switch Case for Lift gm0.org
        switch (liftState) {
            case LIFT_START:
                // In Idle state, wait until Driver 2 right bumper is pressed
                if (gamepad2.right_bumper) {
                    //Extend Lift
                    lift.setHeight(LiftConstants.liftHigh);
                    liftState = LiftState.LIFT_EXTEND;
                }
                break;
            case LIFT_EXTEND:
                //Check if lift has fully extended
                if (Math.abs(leftLift.getCurrentPosition() - LiftConstants.liftHigh) < 10) {
                    //Deploy box
                    lift.extendBox();
                    liftState = LiftState.BOX_EXTEND;
                }
                break;
            case BOX_EXTEND:
                //Wait for servo to reach position
                if (rightServo.getPosition() == LiftConstants.rightBoxReady) {
                    liftState = LiftState.LIFT_DUMP;
                }
                break;
            case LIFT_DUMP:
                //Wait for Driver 2 to press x for release
                if (gamepad2.x) {
                    //Turn on Outtake Servo
                    liftState = LiftState.BOX_RETRACT;
                    //Reset outtake timer
                    liftTimer.reset();
                }
                break;
            case BOX_RETRACT:
                //Wait for pixels to spin out
                if (liftTimer.seconds() >= LiftConstants.dumpTime) {
                    //Turn off Outtake Servo
                    //Retract Box
                    lift.retractBox();
                    liftState = LiftState.LIFT_RETRACT;
                }
                break;
            case LIFT_RETRACT:
                // Wait for servo to return to Idle
                if (rightServo.getPosition() == LiftConstants.rightBoxIdle) {
                    //Retract Lift
                    lift.setHeight(LiftConstants.liftRetracted);
                    liftState = LiftState.LIFT_RETRACTED;
                }
                break;
            case LIFT_RETRACTED:
                //Wait for Lift to return to idle
                if (Math.abs(leftLift.getCurrentPosition() - LiftConstants.liftRetracted) < 10) {
                    liftState = LiftState.LIFT_START;
                }
                break;
            default:
                //Should never happen but just in case
                liftState = LiftState.LIFT_START;
        }
    }

    @Override
    public void stop() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);


    }

}


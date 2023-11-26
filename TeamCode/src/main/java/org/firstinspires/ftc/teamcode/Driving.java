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
    private DcMotor winch = null;

    private CRServo IOservo = null; // intake and outtake servo
    private Servo rightServo = null;
    private Servo leftServo =null;
    private Servo winchServo = null;

    private DigitalChannel limitswitch = null;

    private DcMotor Intake = null;

    private int liftHeight = 0;

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
    //Enum to represent winch state
    private enum WinchState {
        IDLE,
        HOOK_ON,
        EXTEND,
        IDLE_HIGH,
        HOOK_OFF,
        RETRACT,
    }

    //Timer for waiting for pixels to spin out
    ElapsedTime liftTimer = new ElapsedTime();

    //Timer for winch to hang
    ElapsedTime winchTimer = new ElapsedTime();
    //Initial Lift Position
    LiftState liftState = LiftState.LIFT_START;
    WinchState winchState = WinchState.IDLE;

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
        winch = hardwareMap.get(DcMotor.class, "winch");

        IOservo = hardwareMap.get(CRServo.class, "IOservo");
        Intake=hardwareMap.get(DcMotor.class, "intake");
        rightServo = hardwareMap.get(Servo.class, "Right_outtake");
        leftServo = hardwareMap.get(Servo.class, "Left_outtake");
        winchServo = hardwareMap.get(Servo.class, "winch_servo");
        limitswitch = hardwareMap.get(DigitalChannel.class, "limitswitch");




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
        leftFrontPower = Range.clip(drive + turn + strafe, -1, 1);
        rightFrontPower = Range.clip(drive - turn - strafe, -1, 1);
        leftBackPower = Range.clip(drive + turn - strafe, -1, 1);
        rightBackPower = Range.clip(drive - turn + strafe, -1, 1);
        //Driving Slow Mode
        if (gamepad1.right_bumper) {
            leftFrontPower /= 2;
            leftBackPower /= 2;
            rightFrontPower /= 2;
            rightBackPower /= 2;
        }
        //Intake and Reject
        if (gamepad2.a) {
            Intake.setPower(1);
            IOservo.setPower(1);
        } else if (gamepad2.b) {
            Intake.setPower(-1);
            IOservo.setPower(1);


        } else {
            Intake.setPower(0);
            IOservo.setPower(0);
        }

        telemetry.addData("state",limitswitch.getState());

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        //Claw Code: Opens with GP2 X and opens less when past vertical position
        // BIGGER CLOSES MORE*********************

        //Winch Code
        telemetry.addData("WinchServo", winchServo.getPosition());

        //Switch Case for Lift gm0.org
        switch (liftState) {
            case LIFT_START:
                lift.retractBox();
                // In Idle state, wait until Driver 2 right bumper is pressed
                //Extend lift
                if (gamepad2.right_bumper) {
                    liftState = LiftState.LIFT_EXTEND;
                    liftHeight = LiftConstants.liftHigh;
                } else if (gamepad2.right_trigger>= 0.9) {
                    liftState = LiftState.LIFT_EXTEND;
                    liftHeight = LiftConstants.liftMedium;
                } else if (gamepad2.left_bumper) {
                    liftState = LiftState.LIFT_EXTEND;
                    liftHeight = LiftConstants.liftLow;
                }
                break;
            case LIFT_EXTEND:
                //Check if lift has fully extended
                if (Math.abs(leftLift.getCurrentPosition() - liftHeight) < 20) {
                    //Deploy box
                    lift.extendBox();
                    liftState = LiftState.BOX_EXTEND;
                }
                break;
            case BOX_EXTEND:
                //Wait for servo to reach position
                if (rightServo.getPosition() == LiftConstants.BoxReady) {
                    liftState = LiftState.LIFT_DUMP;
                }
                break;
            case LIFT_DUMP:
                //Adjustment midroutine in case Driver puts in wrong input
                if (gamepad2.right_bumper) {
                    liftHeight = LiftConstants.liftHigh;
                } else if (gamepad2.right_trigger>= 0.9) {
                    liftHeight = LiftConstants.liftMedium;
                } else if (gamepad2.left_bumper) {
                    liftHeight = LiftConstants.liftLow;
                }
                //Wait for Driver 2 to press x for release
                if (gamepad2.x) {
                    liftState = LiftState.BOX_RETRACT;
                    //Reset outtake timer
                    //liftTimer.reset();
                }
                break;
            case BOX_RETRACT:
                //Turn on Outtake Servo
                IOservo.setPower(-1);
                //Wait for pixels to spin out
               // if (liftTimer.seconds() >= LiftConstants.dumpTime) {
                    //Turn off Outtake Servo
                   // IOservo.setPower(0);
                    //Retract Box
                  //  lift.retractBox();
                   // liftState = LiftState.LIFT_RETRACT;
              //  }
                if (gamepad2.x) {
                    IOservo.setPower(-1);
                } else
                {
                    liftState = LiftState.LIFT_RETRACT;
                    IOservo.setPower(0);
                    lift.retractBox();
                }
                break;
            case LIFT_RETRACT:
                // Wait for servo to return to Idle
                if (rightServo.getPosition() == LiftConstants.BoxIdle) {
                    liftState = LiftState.LIFT_RETRACTED;
                    //Retract Lift
                    liftHeight = LiftConstants.liftRetracted;
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
        telemetry.addData("liftHeight", liftHeight);
        //Winch Finite State Machine

        switch (winchState) {
            case IDLE:
                winchServo.setPosition(0.25);
                if (gamepad1.a) {
                    winchServo.setPosition(0.22);
                    winchState = WinchState.HOOK_ON;
                    winchTimer.reset();
                }
                break;
            case HOOK_ON:
                if (winchTimer.seconds() > 0.4) {
                    winchServo.setPosition(0.55);
                    winchState = winchState.EXTEND;
                    liftHeight = LiftConstants.liftWinch;
                }
                break;
            case EXTEND:
                if (Math.abs(leftLift.getCurrentPosition() - LiftConstants.liftWinch) < 10) {
                    winchState = winchState.IDLE_HIGH;
                }
                break;
            case IDLE_HIGH:
                if (gamepad1.a) {

                    liftHeight = LiftConstants.liftRetracted;
                    winchState = winchState.HOOK_OFF;
                }
                break;
            case HOOK_OFF:
                if (gamepad1.a)
                    winch.setPower(-1);
                else
                    winch.setPower(0);
                winchServo.setPosition(0.25);
                break;
        }
        if (gamepad2.dpad_down){
            if (limitswitch.getState()){
                liftHeight = -400;
            } else {
                liftHeight = leftLift.getCurrentPosition();
                leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }

        lift.setHeight(liftHeight);
    }


    @Override
    public void stop() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);


    }

}


package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.LiftConstants.liftHang;
import static org.firstinspires.ftc.teamcode.LiftConstants.liftLow;
import static org.firstinspires.ftc.teamcode.LiftConstants.liftRetracted;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

    private Servo  DIservo = null;
    private Servo winchServo = null;

    private DistanceSensor distance = null;

    private DigitalChannel limitswitch = null;
    private DigitalChannel boxBeam = null;
    private boolean isOn = false;

    private int liftOffset = 0;
    private DcMotor Intake = null;

    private double previousEstimate;
    private double currentEstimate;
    private double a;
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
    private enum HangState {
        LIFT_START,
        LIFT_EXTEND,
        BOX_EXTEND,
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
    //Presets for fine adjustment in lift
    int heightAdjust = 0;
    //Timer for waiting for pixels to spin out
    ElapsedTime liftTimer = new ElapsedTime();

    //Timer for winch to hang
    ElapsedTime winchTimer = new ElapsedTime();
    //Initial Lift Position
    LiftState liftState = LiftState.LIFT_START;
    WinchState winchState = WinchState.IDLE;
    HangState hangState = HangState.LIFT_START;
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
        winchServo = hardwareMap.get(Servo.class, "winch_servo");
        limitswitch = hardwareMap.get(DigitalChannel.class, "limitswitch");
        boxBeam = hardwareMap.get(DigitalChannel.class, "box_beam");
       distance = hardwareMap.get(DistanceSensor.class, "Distance");
       DIservo = hardwareMap.get(Servo.class, "DIservo");




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
        DIservo.setDirection(Servo.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("status", "Initialized");
        //Box servo moving in init
        lift.retractBox();
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
        if (gamepad2.left_stick_button) {
            liftState = LiftState.BOX_RETRACT;
        }
        //Intake and Reject
        if (gamepad2.a) {
            Intake.setPower(1);
            IOservo.setPower(1);
        } else if (gamepad2.b) {
            Intake.setPower(-1);
            IOservo.setPower(1);
            //Break Beam Driver feedback via controller rumble
            if (!boxBeam.getState()) {
                gamepad2.rumble(50);
                gamepad1.rumble(50);
            }

        } else {
            Intake.setPower(0);
            IOservo.setPower(0);
        }

        if (gamepad2.left_stick_y>0.3) {
            DIservo.setPosition(0.5);
        } else  {
            DIservo.setPosition(0);
        }

        telemetry.addData("state",limitswitch.getState());


        //Claw Code: Opens with GP2 X and opens less when past vertical position
        // BIGGER CLOSES MORE*********************

        //Switch Case for Lift gm0.org
        switch (liftState) {
            case LIFT_START:
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
                if (Math.abs(leftLift.getCurrentPosition() - liftHeight - liftOffset) < 20) {
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

                //Further adjustment from presets
//                if(-gamepad2.left_stick_y >= 0.5 && heightAdjust == 0) {
//                    liftHeight += 200;
//                    heightAdjust = 1;
//                }
//                if(-gamepad2.left_stick_y <= 0.5 && heightAdjust == 1) {
//                    liftHeight -= 200;
//                    heightAdjust = 0;
//                }

                if (gamepad2.x) {
                    //Turn on Outtake Servo
                    IOservo.setPower(-1);
                    heightAdjust = 1;
                    liftState = LiftState.BOX_RETRACT;
                }
                break;
            case BOX_RETRACT:
                if (gamepad2.right_bumper) {
                    liftHeight = LiftConstants.liftHigh;
                } else if (gamepad2.right_trigger>= 0.9) {
                    liftHeight = LiftConstants.liftMedium;
                } else if (gamepad2.left_bumper) {
                    liftHeight = LiftConstants.liftLow;
                }
                    if (gamepad2.x) {
                        IOservo.setPower(-1);
                    } else
                        IOservo.setPower(0);
                    if (gamepad2.left_trigger >= 0.8) {
                        IOservo.setPower(0);
                        lift.retractBox();
                        liftState = LiftState.LIFT_RETRACT;
                    }
                break;
            case LIFT_RETRACT:
                // Wait for servo to return to Idle
                if (rightServo.getPosition() == LiftConstants.BoxIdle) {
                    liftState = LiftState.LIFT_RETRACTED;
                    //Retract Lift
                    liftHeight = liftRetracted;
                }
                break;
            case LIFT_RETRACTED:
                //Wait for Lift to return to idle
                if (Math.abs(leftLift.getCurrentPosition() - liftRetracted - liftOffset) < 10) {
                    liftState = LiftState.LIFT_START;
                }
                break;
            default:
                //Should never happen but just in case
                liftState = LiftState.LIFT_START;
        }
        //Telemetry for liftHeight based on state machine (Not actual position, but what height the program is telling the robot)
        telemetry.addData("liftHeight", liftHeight);
        telemetry.addData("boxBeam", boxBeam.getState());

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
                    winchServo.setPosition(0.62);
                    winchState = WinchState.EXTEND;
                    liftHeight = LiftConstants.liftWinch;
                }
                break;
            case EXTEND:
                if (Math.abs(leftLift.getCurrentPosition() - LiftConstants.liftWinch - liftOffset) < 10) {
                    winchState = WinchState.IDLE_HIGH;
                }
                break;
            case IDLE_HIGH:
                if (gamepad1.a) {
                    liftHeight = liftRetracted;
                    winchState = WinchState.HOOK_OFF;
                }
                break;
            case HOOK_OFF:
                if (gamepad1.a)
                    winch.setPower(-1);
                else
                    winch.setPower(0);
                break;
        }
        //For hanging with slides
        switch (hangState) {
            case LIFT_START:
                //Driver 2 Dpad starts sequence
                if (gamepad2.dpad_up) {
                    hangState = HangState.LIFT_EXTEND;
                    //Extend lift
                    liftHeight = LiftConstants.liftHang2;
                }
                break;
            case LIFT_EXTEND:
                //Check if lift has fully extended
                if (Math.abs(leftLift.getCurrentPosition() - liftHeight - liftOffset) < 20) {
                    //Angle box out of the way
                    lift.hangBox();
                    if (gamepad2.dpad_up)
                        hangState = HangState.LIFT_RETRACT;
                }
                break;
            case LIFT_RETRACT:
                // Wait for dpad_up to retract lift and hang
                //Goes straight back to start in case it gets stuck and can't retract all the way
                liftHeight = liftHang;
                if (!gamepad2.dpad_up) {
                    hangState = HangState.LIFT_START;
                    //Retract Lift
                }
                break;
            default:
                //Should never happen but just in case
                liftState = LiftState.LIFT_START;
        }

        //Rezeroing the slides using limit switch
        //Adds an offset value to the liftHeight based on the left motors difference
        if (gamepad2.dpad_down){
            if (limitswitch.getState()){
                liftHeight = -400;
                isOn = false;
            } else if (!isOn && !limitswitch.getState()){
                isOn = true;
                liftHeight = 0;
                //Adding 25 to relieve the stress that comes from slamming the box down lol
                liftOffset = leftLift.getCurrentPosition()+25;
            }
        }

        //Actually setting the lift height; kept out of the finite state machine so that the PID continues to update
        lift.setHeight(liftHeight + liftOffset);


        double value = distance.getDistance(DistanceUnit.INCH);
        if(currentEstimate < 3) {
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
        }
        else {
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
        }
       telemetry.addData("Distance: ", currentEstimate);
        a = 0.8; // a can be anything from 0 < a < 1
        previousEstimate = 0;
        currentEstimate = 0;
        currentEstimate = (a * previousEstimate) + (1-a) * value;
        previousEstimate = currentEstimate;

    }


    @Override
    public void stop() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);


    }

}



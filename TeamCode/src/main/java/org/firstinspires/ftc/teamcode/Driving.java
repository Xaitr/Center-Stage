package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.LiftConstants.StackMuncher2;
import static org.firstinspires.ftc.teamcode.LiftConstants.droneLift;
import static org.firstinspires.ftc.teamcode.LiftConstants.liftHang;
import static org.firstinspires.ftc.teamcode.LiftConstants.liftLow;
import static org.firstinspires.ftc.teamcode.LiftConstants.liftRetracted;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.hardware.IMU;
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


    private Servo  DIservo = null;
    private Servo preDropRight = null;
    private Servo preDropLeft = null;
    private Servo drone = null;

    private RevBlinkinLedDriver Blinky = null;


    private DigitalChannel limitswitch = null;
    private DigitalChannel boxBeam = null;
    private boolean isOn = false;

    private boolean boxDrop = false;

    private boolean liftIncrease = false;

    private boolean liftDecrease = false;

    private int liftOffset = 0;
    private DcMotor Intake = null;
    private int liftHeight = 0;



    private double DIservoposition  = 0;
//funny little comment
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

    private enum  dronestate {

        LIFT_START,
        LIFT_EXTEND,
        DRONE_LAUNCH,
        LIFT_RETRACT,


    }
    ElapsedTime droneTimer = new ElapsedTime();
    dronestate droneState = dronestate.LIFT_START;
    private IMU imu = null;
    //Presets for fine adjustment in lift
    int heightAdjust = 0;
    //Timer for waiting for pixels to spin out
    ElapsedTime liftTimer = new ElapsedTime();

    //Initial Lift Position
    LiftState liftState = LiftState.LIFT_START;
    HangState hangState = HangState.LIFT_START;

    boolean leftStickButtonDown = false;
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
        limitswitch = hardwareMap.get(DigitalChannel.class, "limitswitch");
        boxBeam = hardwareMap.get(DigitalChannel.class, "box_beam");
       DIservo = hardwareMap.get(Servo.class, "DIservo");
       preDropRight = hardwareMap.get(Servo.class, "preDropRight");
       preDropLeft = hardwareMap.get(Servo.class, "preDropLeft");
       drone = hardwareMap.get(Servo.class, "drone");
       Blinky = hardwareMap.get(RevBlinkinLedDriver.class, "Blinky");




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
    DIservo.setPosition(LiftConstants.StackMuncherReturn);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(RevOrientation));
        imu.resetYaw();
    }




    private double incrementDiservo(double currentPosition) {
        if (currentPosition == LiftConstants.StackMuncherReturn) {
            return LiftConstants.StackMuncher1;
        } else if (currentPosition == LiftConstants.StackMuncher1) {
            return LiftConstants.StackMuncher2;
        } else if (currentPosition == LiftConstants.StackMuncher2) {
            return LiftConstants.StackMuncher3;
        } else if (currentPosition == LiftConstants.StackMuncher3) {
            return LiftConstants.StackMuncher4;
        } else if (currentPosition == LiftConstants.StackMuncher4) {
            return LiftConstants.StackMuncher5;
        } else {
            return LiftConstants.StackMuncherReturn;
        }
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
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        if (gamepad2.dpad_left) {
            liftState = LiftState.BOX_RETRACT;
        }
        //Intake and Reject
        if (gamepad2.a) {
            Intake.setPower(1);
            IOservo.setPower(1);
            boxDrop = false;
        } else if (gamepad2.b) {
            Intake.setPower(-1);
            IOservo.setPower(1);
            //Break Beam Driver feedback via controller rumble
            if (!boxBeam.getState()) {
                gamepad1.rumble(200);
                gamepad2.rumble(200);
            }
            boxDrop = false;

        } else if (!boxDrop) {
            Intake.setPower(0);
            IOservo.setPower(0);

        }
        telemetry.addData("boxDrop", boxDrop);

        if (!boxBeam.getState()) {
            Blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
        }
        else {
            Blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
        if (gamepad2.left_stick_button) {
              leftStickButtonDown = true;
              Blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
               } else {
             if (leftStickButtonDown) {
                 DIservoposition = incrementDiservo(DIservoposition);
                 DIservo.setPosition(DIservoposition);
                leftStickButtonDown = false;
                }
              }

             if (gamepad2.left_stick_y<-0.6) {
                 DIservo.setPosition(LiftConstants.StackMuncherReturn);
                 DIservoposition = LiftConstants.StackMuncherReturn;
             }





             telemetry.addData("state",limitswitch.getState());


        //Claw Code: Opens with GP2 X and opens less when past vertical position
        // BIGGER CLOSES MORE*********************

        telemetry.addData("IMU:", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        if (gamepad1.left_bumper) {
            drone.setPosition(0.6);
        }


        //Switch Case for Lift gm0.org
        switch (liftState) {
            case LIFT_START:
                // In Idle state, wait until Driver 2 right bumper is pressed
                //Extend lift
                if (gamepad2.right_bumper) {
                    liftState = LiftState.LIFT_EXTEND;
                    liftHeight = LiftConstants.liftHigh;
                } else if (gamepad2.right_trigger >= 0.9) {
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
                //Adjustment mid match in case Driver puts in wrong input
                if (gamepad2.right_bumper) {
                    liftHeight = LiftConstants.liftHigh;
                } else if (gamepad2.right_trigger >= 0.9) {
                    liftHeight = LiftConstants.liftMedium;
                } else if (gamepad2.left_bumper) {
                    liftHeight = LiftConstants.liftLow;
                }
                if (gamepad2.right_stick_y < -0.6 && !liftIncrease) { // lift goes up
                    liftHeight += 225;
                    liftIncrease = true;
                } else if (gamepad2.right_stick_y > -0.6 ) {
                    liftIncrease = false;
                }

                if (gamepad2.right_stick_y > 0.6 && !liftDecrease) {
                    liftHeight -= 225;
                    liftDecrease = true;
                } else if (gamepad2.right_stick_y < 0.6) {
                    liftDecrease = false;
                }

                //Servo Drop: Changes boxDrop variable too stop intake code
                if (gamepad2.right_stick_button) {
                    //Turn on Outtake Servo
                    IOservo.setPower(-1);
                    boxDrop = true;
                   // heightAdjust = 1;
                } else{
                    boxDrop = false;
                    IOservo.setPower(0);
                }

                if (gamepad2.left_trigger >= 0.8) {
                    IOservo.setPower(0);
                    lift.retractBox();
                    liftState = LiftState.LIFT_RETRACT;
                }
                break;
            case LIFT_RETRACT:
                // Wait for servo to return to Idle
                lift.retractBox();
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
                    if (gamepad2.dpad_down)
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

        // drone launch
        switch (droneState) {
            case LIFT_START:
                //Driver 2 Dpad starts sequence
                if (gamepad1.dpad_right) {
                    droneState = droneState.LIFT_EXTEND;
                    //Extend lift
                    liftHeight = LiftConstants.droneLift;
                }
                break;
            case LIFT_EXTEND:
                //Check if lift has fully extended
                if (Math.abs(leftLift.getCurrentPosition() - liftHeight - liftOffset) < 20) {
                    //Angle box out of the way
                    lift.droneBox();
                    if (gamepad1.dpad_left) {
                        droneState = droneState.DRONE_LAUNCH;
                        drone.setPosition(0.6);
                        droneTimer.reset();
                    }
                }
                break;
            case DRONE_LAUNCH:
                if  (droneTimer.seconds() > 0.5) {
                    droneState = droneState.LIFT_RETRACT;
                lift.retractBox();
                droneTimer.reset();
                // this launches the drone and has a timer so the box does not hit the drone
                }
                break;
                case LIFT_RETRACT:
                // Wait for dpad_up to retract lift and hang
                //Goes straight back to start in case it gets stuck and can't retract all the way

                if (droneTimer.seconds() > 0.3) {
                    droneState = droneState.LIFT_START;
                    //Retract Lift
                    liftHeight = liftRetracted;

                }
                break;
            default:
                //Should never happen but just in case
                droneState = droneState.LIFT_START;
        }

        //SAFETY
        if (gamepad2.y)
            liftState = LiftState.LIFT_START;

        //Rezeroing the slides using limit switch
        //Adds an offset value to the liftHeight based on the left motors difference
//        if (gamepad2.dpad_down){
//            if (limitswitch.getState()){
//                liftHeight = -400;
//                isOn = false;
//            } else if (!isOn && !limitswitch.getState()){
//                isOn = true;
//                liftHeight = 0;
//                //Adding 25 to relieve the stress that comes from slamming the box down lol
//                liftOffset = leftLift.getCurrentPosition()+25;
//            }
//        }

        //Actually setting the lift height; kept out of the finite state machine so that the PID continues to update
        lift.setHeight(liftHeight + liftOffset);
    }




    @Override
    public void stop() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);


    }

}



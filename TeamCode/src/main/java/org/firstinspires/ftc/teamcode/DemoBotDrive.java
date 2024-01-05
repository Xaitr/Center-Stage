//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.DigitalChannel;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.TouchSensor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//import com.qualcomm.robotcore.hardware.CRServo;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//
//@TeleOp
//public class DemoBotDrive extends OpMode
//{
//    private DcMotor leftFrontDrive = null;
//    private DcMotor leftBackDrive = null;
//    private DcMotor rightFrontDrive = null;
//    private DcMotor rightBackDrive = null;
////    private DcMotor winchDrive = null;
////
////    private Servo claw = null;
//
//    private DistanceSensor distance = null;
//
//
//
//    // Enum to represent lift state
//
//
//
//    @Override
//    public void init() {
//
//
//        //Declare variables for phone to recognise//
//
//        //names on the config
//
//        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front");
//        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
//        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back");
//        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");
////        winchDrive = hardwareMap.get(DcMotor.class, "winch_motor");
////        claw = hardwareMap.get(Servo.class,"claw");
//        distance = hardwareMap.get(DistanceSensor.class, "Distance");
//
//        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////        winchDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
//
//        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        telemetry.addData("status", "Initialized");
//    }
//
//
//    //Set variables//
//    @Override
//    public void loop() {
//        double leftFrontPower;
//        double rightFrontPower;
//        double leftBackPower;
//        double rightBackPower;
//        double winchPower;
//
//
//        double drive = -gamepad1.left_stick_y;
//        double strafe = gamepad1.left_stick_x;
//        double turn = gamepad1.right_stick_x;
//        leftFrontPower = Range.clip(drive + turn - strafe, -0.5, 0.5);
//        rightFrontPower = Range.clip(drive - turn + strafe, -0.5, 0.5);
//        leftBackPower = Range.clip(drive + turn + strafe, -0.5, 0.5);
//        rightBackPower = Range.clip(drive - turn - strafe, -0.5, 0.5);
//        winchPower = Range.clip(-gamepad2.left_stick_y, -0.5, 0.5);
//
//        if (gamepad1.right_bumper) {
//            leftFrontPower /= 2;
//            leftBackPower /= 2;
//            rightFrontPower /= 2;
//            rightBackPower /= 2;
//
//        }
////        if (gamepad2.left_bumper){
////            claw.setPosition(0);
////        }
////        else if (gamepad2.right_bumper){
////            claw.setPosition(0.5);
////        }
///*
//       if (gamepad2.x) {
//           outake.setHeight(100);
////           IOservo.setPower(-1);
//       }
//       if (gamepad2.right_bumper) {
//           RightServo.setPosition(0);
//           LeftServo.setPosition(0);
//       }
//       else if (gamepad2.left_bumper) {
//           RightServo.setPosition(0.25);
//           LeftServo.setPosition(0.25);
//       }
//*/
//
//        leftFrontDrive.setPower(leftFrontPower);
//        rightFrontDrive.setPower(rightFrontPower);
//        leftBackDrive.setPower(leftBackPower);
//        rightBackDrive.setPower(rightBackPower);
////        winchDrive.setPower(winchPower);
//
//        double value = distance.getDistance(DistanceUnit.INCH);
//        if(distance.getDistance(DistanceUnit.INCH) > 10);
//        telemetry.addData("Distance: ", value);
//        telemetry.update();
//




        //Claw Code: Opens with GP2 X and opens less when past vertical position
        // BIGGER CLOSES MORE*********************

        //Switch Case for Lift gm0.org
//    }
//        @Override
//        public void stop() {
//            leftFrontDrive.setPower(0);
//            rightFrontDrive.setPower(0);
//            leftBackDrive.setPower(0);
//            rightBackDrive.setPower(0);
////            winchDrive.setPower(0);
//
//
//        }


//}


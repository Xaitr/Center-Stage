package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PidControl2 {
    DcMotorEx leftLift;
    DcMotorEx rightLift;

    double integralSum =0;
    double Kp =0.007; // 0.045
    double Ki =0;
    double Kd = 0.000001; // 0.0000038

//0.000001
    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private Servo rightServo, leftServo, wristServo = null;

    public void init(HardwareMap hardwareMap) {
        leftLift = hardwareMap.get(DcMotorEx.class, "left_lift");
        rightLift = hardwareMap.get(DcMotorEx.class, "right_lift");

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        leftLift.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);


        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightServo = hardwareMap.get(Servo.class, "Right_outtake");
        leftServo = hardwareMap.get(Servo.class, "Left_outtake");

        rightServo.setDirection(Servo.Direction.REVERSE);

    }
    public double PIDControl(double reference, double state) {
        double error = reference - state;
        integralSum = error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }
    public void setHeight(double height) {
        double power = PIDControl(height, leftLift.getCurrentPosition());
        leftLift.setPower(power);
        rightLift.setPower(power);

    }
    public void extendBox() {
        rightServo.setPosition(LiftConstants.BoxReady);
        leftServo.setPosition(LiftConstants.BoxReady);
    }
    public void retractBox() {
        rightServo.setPosition(LiftConstants.BoxIdle);
        leftServo.setPosition(LiftConstants.BoxIdle);
    }
    public void AutoBoxReady() {
        rightServo.setPosition(LiftConstants.AutoBoxReady);
        leftServo.setPosition(LiftConstants.AutoBoxReady);
    }
    public void hangBox() {
        rightServo.setPosition(LiftConstants.hang);
        leftServo.setPosition(LiftConstants.hang);
    }
    public void disableMotors() {
        leftLift.setPower(0);
        rightLift.setPower(0);
    }
public void droneBox () {
        rightServo.setPosition(LiftConstants.droneAngle);
        leftServo.setPosition(LiftConstants.droneAngle);
}
    //Rotates the wrist clockwise to the next wrist position
    public double wristRight(double currentPosition) {
        if (currentPosition == LiftConstants.wristRight1)
            return LiftConstants.wristMiddle1;
        else if (currentPosition == LiftConstants.wristMiddle1)
            return LiftConstants.wristLeft1;
        else if (currentPosition == LiftConstants.wristLeft1)
            return LiftConstants.wristLeft2;
        else if (currentPosition == LiftConstants.wristLeft2)
            return LiftConstants.wristMiddle2;
        else if (currentPosition == LiftConstants.wristMiddle2)
            return LiftConstants.wristRight2;
        else {
            return LiftConstants.wristRight1;
        }
    }

    //Rotates the wrist counter-clockwise to the next wrist position
    public double wristLeft(double currentPosition) {
        if (currentPosition == LiftConstants.wristRight1)
            return LiftConstants.wristRight2;
        else if (currentPosition == LiftConstants.wristRight2)
            return LiftConstants.wristMiddle2;
        else if (currentPosition == LiftConstants.wristMiddle2)
            return LiftConstants.wristLeft2;
        else if (currentPosition == LiftConstants.wristLeft2)
            return LiftConstants.wristLeft1;
        else if (currentPosition == LiftConstants.wristLeft1)
            return LiftConstants.wristMiddle1;
        else {
            return LiftConstants.wristRight1;
        }
    }


}





package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PidControl2 extends LinearOpMode {
    DcMotorEx motor;

    double integralSum =0;
    double Kp =0;
    double Ki =0;
    double Kd = 0;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "Motor");
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while  (opModeIsActive()) {
            double power = PIDControl( 100, motor.getCurrentPosition());
            motor.setPower(power);
        }
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
}



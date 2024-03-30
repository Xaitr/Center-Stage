package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.LiftConstants.StackMuncher1;
import static org.firstinspires.ftc.teamcode.LiftConstants.StackMuncher2;
import static org.firstinspires.ftc.teamcode.LiftConstants.StackMuncher3;
import static org.firstinspires.ftc.teamcode.LiftConstants.StackMuncher4;
import static org.firstinspires.ftc.teamcode.LiftConstants.StackMuncher5;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    private DcMotor intake;
    private Servo DIservo;

    public void init(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        DIservo = hardwareMap.get(Servo.class, "DIservo");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void Reject() {
        intake.setPower(-0.40);
    }

    public void RejectStronger() {
        intake.setPower(-0.44);
    }

    public void RejectOff() {
        intake.setPower(0);
    }

    public void DeployIntakeFirst() {

    }


    }


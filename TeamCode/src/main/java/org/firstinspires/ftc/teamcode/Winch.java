package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp
public class Winch extends OpMode {
    DcMotor winch = null;
    @Override
    public void init() {
        winch = hardwareMap.get(DcMotor.class, "winch");
    }
    public void loop() {
        if (gamepad1.a)
            winch.setPower(1);
        else if (gamepad1.b)
            winch.setPower(-1);
        else
            winch.setPower(0);
    }
}

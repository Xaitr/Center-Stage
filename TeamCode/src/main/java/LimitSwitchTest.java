package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Disabled
@TeleOp
public class LimitSwitchTest extends OpMode{

    private DigitalChannel digIn;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    public void init() {
        digIn = hardwareMap.get(DigitalChannel.class, "digin");
    }
    @Override
    public void loop() {
        if (digIn.getState()){
            telemetry.addLine("Im Pressed! :D");
        } else {
            telemetry.addLine("Im not Pressed ):");
        }
    }
}
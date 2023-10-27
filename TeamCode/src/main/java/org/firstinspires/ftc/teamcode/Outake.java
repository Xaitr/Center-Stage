package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Outake {
    private DcMotorEx lift_left;
    private DcMotorEx lift_right;

    private CRServo IOServo;  // outtake servo

    double lastError = 0;



    ElapsedTime timer = new ElapsedTime();


    private static void setTimeout(Runnable runnable, int delay){
        new Thread(() -> {
            try {
                Thread.sleep(delay);
                runnable.run();
            }
            catch (Exception e){
                System.err.println(e);
            }
        }).start();
    }

    public void init(HardwareMap hardwareMap){

        lift_left = hardwareMap.get(DcMotorEx.class, "left_lift");
        lift_right = hardwareMap.get(DcMotorEx.class, "right_lift");
        IOServo = hardwareMap.get(CRServo.class, "Reverse outtake servo");



        lift_left.setTargetPosition(0);
        lift_right.setTargetPosition(0);

        lift_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        lift_left.setDirection(DcMotorSimple.Direction.REVERSE);
        lift_right.setDirection(DcMotorSimple.Direction.FORWARD);


        lift_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



    }

    public void setHeight(double height){
        double ticks_per_mm = (384.5 / 112);
        double value = height * ticks_per_mm;
        int target = (int)value;
        lift_left.setTargetPosition(target);
        lift_left.setVelocity(2785);
        lift_right.setVelocity(2785);
    }


    public void stopslide(){
        lift_left.setVelocity(0);
        lift_right.setVelocity(0);
    }






    public void PIDarm(double angle){
        double ticksPerDegree = (2786.2 / 360);
        double reference = angle * ticksPerDegree;

        double Kp = 0.0008;
        double Ki = 0;
        double Kd = 0;





    }



    public double getHeight(){return lift_left.getCurrentPosition() / (384.5/112);}
    public double getRightHeight(){return lift_right.getCurrentPosition() / (384.5/112);}


}

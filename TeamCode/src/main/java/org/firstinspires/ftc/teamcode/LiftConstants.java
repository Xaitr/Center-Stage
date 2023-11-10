package org.firstinspires.ftc.teamcode;

//Parameters for lift
public class LiftConstants {
    //Lift heights in ticks
    public static double liftHigh = 2000;
    public static double liftRetracted =0;

    //Left and Right servo positions
    // Opposite because they turn opposite directions to move the box the same way
    public static double leftBoxIdle = 0.1;
    public static double rightBoxIdle = 0.9;
    public static double leftBoxReady = 0.3;
    public static double rightBoxReady = 0.7;

    //Time it takes for both pixels to spin out
    public static double dumpTime = 2;
}

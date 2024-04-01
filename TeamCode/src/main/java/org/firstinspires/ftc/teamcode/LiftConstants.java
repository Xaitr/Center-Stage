package org.firstinspires.ftc.teamcode;

//Parameters for lift
public class LiftConstants {
    //Lift heights in ticks
    public static int liftHigh = 1945;
    public static int liftMedium = 1495;
    public static int liftLow = 1045;
    //1100, 1700, 2200 MAX
    public static int liftRetracted =0;
    public static int liftHang = 300;
    public static int liftHang2 = 1700;

    //Left and Right servo positions
    public static double BoxIdle = 0;
    public static double BoxReady = 0.4;
    public static double hang = 0.8;

    public static double droneAngle = 0.74;

    public static int droneLift = 600;

    public static double StackMuncher1 = 0.875;

    public static double StackMuncher2 = 0.90;

    public static double StackMuncher3 = 0.905;

   public static double StackMuncher4 = 0.92;

   public static double StackMuncher5 = 0.935;

   public static double StackMuncherReturn = 0;
    public static double AutoBoxReady = 0.4;

    //Wrist rotations (Right/Left is the side thats higher)
   public static double wristIdle = 0;
   public static double wristMiddle1 = 90*(1/177.5);
   public static double wristRight1 = 30*(1/177.5);
   public static double wristLeft1 = 150*(1/177.5);
   public static double wristMiddle2 = -90*(1/177.5);
   public static double wristRight2 = -30*(1/177.5);
   public static double wristLeft2 = -150*(1/177.5);

    //Time it takes for both pixels to spin out

//    public static double AutoBoxReady = 0.4;
    public static double dumpTime = 1.5;
    public static double singleDump = 0.4;

    //Lift height for winch
    public static int liftWinch = 720;
    public static int liftAuto = 830;
}

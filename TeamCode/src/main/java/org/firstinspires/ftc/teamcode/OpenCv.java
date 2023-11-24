package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class OpenCv extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location {
        LEFT,
        RIGHT,
        MIDDLE,
        NOT_FOUND
    }
    private Location location;

    static final Rect RIGHT_ROI = new Rect(
            new Point(1290, 600),
            new Point(1060, 800));
    static final Rect Middle_ROI = new Rect(
            new Point(760, 480),
            new Point(540, 730));
    static final Rect LEFT_ROI = new Rect(
            new Point(210,550),
            new Point(0,800)
    );
    static double PERCENT_COLOR_THRESHOLD = 0.3;

    public OpenCv (Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(0,50,50);
        Scalar highHSV = new Scalar(150,255,255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);
        Mat middle = mat.submat(Middle_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area();
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area();
        double middleValue = Core.sumElems(middle).val[0] / Middle_ROI.area();

        left.release();
        right.release();
        middle.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Middle raw value", (int) Core.sumElems(middle).val[0]);
        telemetry.addData("Left percentage", Math.round(leftValue) + "%");
        telemetry.addData("Right percentage", Math.round(rightValue) + "%");
        telemetry.addData("Middle percentage", Math.round(middleValue) + "%");

//        boolean stoneLeft = leftValue > PERCENT_COLOR_THRESHOLD;
//        boolean stoneRight = rightValue > PERCENT_COLOR_THRESHOLD;
//        boolean stoneMiddle = middleValue > PERCENT_COLOR_THRESHOLD;

        if (rightValue > leftValue && rightValue > middleValue) {
            location = Location.RIGHT;
            telemetry.addData("Marker Location", "right");
        }
        else if (leftValue > rightValue && leftValue > middleValue) {
            location = Location.LEFT;
            telemetry.addData("Marker Location", "left");
        } else if (middleValue > rightValue && middleValue > leftValue) {
            location = Location.MIDDLE;
            telemetry.addData("Middle", "Placeholder");
        }
        else {
            location = Location.NOT_FOUND;
            telemetry.addData("Marker Location", "Not Found");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorStone = new Scalar(255, 0, 0);
        Scalar colorSkystone = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? colorSkystone:colorStone);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? colorSkystone:colorStone);
        Imgproc.rectangle(mat,Middle_ROI,location == Location.MIDDLE? colorSkystone:colorStone);

        return mat;
    }

    public Location getLocation() {
        return location;
    }
}
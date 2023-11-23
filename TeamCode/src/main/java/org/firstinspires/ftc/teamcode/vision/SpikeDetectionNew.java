package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.constants.AutoMods;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class SpikeDetectionNew implements VisionProcessor {
    //private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
    public static Point LEFT_RED = new Point(185,325);
    public static Point LEFT_BLUE = new Point(50,325);

    public static Point CENTER_RED = new Point(420,300);
    public static Point CENTER_BLUE = new Point(220,300);

    public static Point RIGHT_RED = new Point(580,325);
    public static Point RIGHT_BLUE = new Point(455,325);

    public static Size BOXSIZEC = new Size(50,20);

    private Scalar left;
    private Scalar center;
    private Scalar right;

    private Rect leftRect;
    private Rect centerRect;
    private Rect rightRect;

    private double thresh;
    public double testval = 0;

    Mat modMat = new Mat();
    Mat leftMat;
    Mat centerMat;
    Mat rightMat;


    public enum Position{

        LEFT, RIGHT, CENTER
    }
    public static Position pos = Position.CENTER;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        thresh = AutoMods.teamRed ? 1 : 1;
//        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        if ((AutoMods.isFar && AutoMods.teamRed) || (!AutoMods.isFar && !AutoMods.teamRed)) {


            leftRect = new Rect(LEFT_RED, BOXSIZEC);
            centerRect = new Rect(CENTER_RED, BOXSIZEC);
            rightRect = new Rect(RIGHT_RED, BOXSIZEC);
        } else {
            leftRect = new Rect(LEFT_BLUE, BOXSIZEC);
            centerRect = new Rect(CENTER_BLUE, BOXSIZEC);
            rightRect = new Rect(RIGHT_BLUE, BOXSIZEC);
        }

    }


    public Position getPos() {
        return this.pos;
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {

        input.copyTo(modMat);
        Imgproc.GaussianBlur(modMat, modMat, new Size(5, 5), 0.0);
        Imgproc.cvtColor(modMat, modMat, Imgproc.COLOR_RGB2HSV);
        leftMat = modMat.submat(leftRect);
        centerMat = modMat.submat(centerRect);
        rightMat = modMat.submat(rightRect);

        left = Core.sumElems(leftMat);
        center = Core.sumElems(centerMat);
        right = Core.sumElems(rightMat);

        if ((AutoMods.isFar && AutoMods.teamRed) || (!AutoMods.isFar && !AutoMods.teamRed)) {
                if (left.val[1] / 100000.0 > thresh) {
                    pos = Position.LEFT;
                } else if (center.val[1] / 100000.0 > thresh) {
                    pos = Position.CENTER;
                } else {
                    pos = Position.RIGHT;
                }

        } else {
            if (right.val[1] / 100000.0 > thresh) {
                pos = Position.RIGHT;
            } else if (center.val[1] / 100000.0 > thresh) {
                pos = Position.CENTER;
            } else {
                pos = Position.LEFT;
            }
        }


        if (pos == Position.LEFT) {
            Imgproc.rectangle(input, leftRect, AutoMods.teamRed? new Scalar (255, 0, 0) : new Scalar (0, 0, 255), 2);
            Imgproc.rectangle(input, rightRect, new Scalar (0, 0, 0), 2);
            Imgproc.rectangle(input, centerRect, new Scalar (0, 0, 0), 2);
        } else if (pos == Position.CENTER){
            Imgproc.rectangle(input, leftRect, new Scalar (0, 0, 0), 2);
            Imgproc.rectangle(input, rightRect, new Scalar (0, 0, 0), 2);
            Imgproc.rectangle(input, centerRect, AutoMods.teamRed? new Scalar (255, 0, 0) : new Scalar (0, 0, 255), 2);
        } else {
            Imgproc.rectangle(input, leftRect, new Scalar (0, 0, 0), 2);
            Imgproc.rectangle(input, rightRect, AutoMods.teamRed? new Scalar (255, 0, 0) : new Scalar (0, 0, 255), 2);
            Imgproc.rectangle(input, centerRect, new Scalar (0, 0, 0), 2);
        }


//        Bitmap b = Bitmap.createBitmap(modMat.width(), modMat.height(), Bitmap.Config.RGB_565);
//        Utils.matToBitmap(modMat, b);
//        lastFrame.set(b);

        leftMat.release();
        centerMat.release();
        rightMat.release();

        return input;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }





}

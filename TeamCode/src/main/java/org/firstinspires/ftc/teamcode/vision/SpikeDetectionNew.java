package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.constants.AutoMods;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class SpikeDetectionNew implements VisionProcessor {
    //private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
    public static int LEFT_X = 100;
    public static int LEFT_Y = 100;

    public static int CENTER_X = 300;
    public static int CENTER_Y = 100;

    public static int RIGHT_X = 500;
    public static int RIGHT_Y = 100;

    public static int WIDTH = 40;
    public static int HEIGHT = 10;

    private Scalar left;
    private Scalar center;
    private Scalar right;

    private Rect leftRect = new Rect(LEFT_X, LEFT_Y, WIDTH, HEIGHT);
    private Rect centerRect = new Rect(CENTER_X, CENTER_Y, WIDTH, HEIGHT);
    private Rect rightRect = new Rect(RIGHT_X, RIGHT_Y, WIDTH, HEIGHT);

    private double maxColor;

    private boolean isRed;



    Mat modMat = new Mat();
    Mat leftMat;
    Mat centerMat;
    Mat rightMat;


    public enum Position{

        LEFT, RIGHT, CENTER
    }
    public static Position pos = Position.RIGHT;

    public SpikeDetectionNew(boolean red) {
        super();
        isRed = red;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
//        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));

    }


    public Position getPos() {
        return this.pos;
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {

        input.copyTo(modMat);
        Imgproc.GaussianBlur(modMat, modMat, new Size(5, 5), 0.0);
        leftMat = modMat.submat(leftRect);
        centerMat = modMat.submat(centerRect);
        rightMat = modMat.submat(rightRect);

        left = Core.sumElems(leftMat);
        center = Core.sumElems(centerMat);
        right = Core.sumElems(rightMat);

        maxColor = Math.max(left.val[isRed? 0 : 2], Math.max(center.val[isRed? 0 : 2], right.val[isRed? 0 : 2]));

        if (maxColor == left.val[isRed? 0 : 2]) {
            pos = Position.LEFT;
            Imgproc.rectangle(input, leftRect, isRed? new Scalar (255, 0, 0) : new Scalar (0, 0, 255), 2);
            Imgproc.rectangle(input, rightRect, new Scalar (0, 0, 0), 2);
            Imgproc.rectangle(input, centerRect, new Scalar (0, 0, 0), 2);
        } else if (maxColor == center.val[isRed? 0 : 2]){
            pos = Position.CENTER;
            Imgproc.rectangle(input, leftRect, new Scalar (0, 0, 0), 2);
            Imgproc.rectangle(input, rightRect, new Scalar (0, 0, 0), 2);
            Imgproc.rectangle(input, centerRect, isRed? new Scalar (255, 0, 0) : new Scalar (0, 0, 255), 2);
        } else {
            pos = Position.RIGHT;
            Imgproc.rectangle(input, leftRect, new Scalar (0, 0, 0), 2);
            Imgproc.rectangle(input, rightRect, isRed? new Scalar (255, 0, 0) : new Scalar (0, 0, 255), 2);
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

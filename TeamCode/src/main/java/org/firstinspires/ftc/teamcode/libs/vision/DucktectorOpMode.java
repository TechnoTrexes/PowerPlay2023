package org.firstinspires.ftc.teamcode.libs.vision;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Disabled
@TeleOp(name= "DucktectorOpMode", group="LinearOpMode")
public class DucktectorOpMode extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static double rectWidth = 1.2f/8f; //1.2f/8f
    private static double rectHeight = 0.6f/8f; //.6f/8f

    private static double[] leftPos = {0.2f/4f, 1.2f/4f}; // 0.75,1/4
    private static double[] midPos = {0.95f/2f, 1.2f/4f};//0 = col, 1 = row //1/2
    private static double[] rightPos = {3.73f/4f, 1.2f/4f}; //3.25 value change based on closeness of tapes (increase if tapes r closer)
    //moves all rectangles right or left by amount. units are in ratio to monitor. 3.6 more accurate value

    private static final int rows = 640;
    private static final int cols = 480;

    private OpenCvCamera phoneCam;
    private OpenCvCameraRotation camRotation = OpenCvCameraRotation.SIDEWAYS_LEFT;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        phoneCam.openCameraDevice();
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, camRotation);//display on RC

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            telemetry.addData("Values", valLeft + "   " + valMid + "   " + valRight);
            telemetry.update();
            sleep(100);
        }
    }

    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline {
        Mat yCbCr = new Mat();
        Mat threshold = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage {
            CB, //includes outlines
            THRESHOLD, //b&w
            RAW_IMAGE //displays raw view
        }

        private Stage stageToRenderToViewport = Stage.CB;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped() {
            int nextStageOrdinal = stageToRenderToViewport.ordinal() + 1;
            if(nextStageOrdinal >= stages.length) {
                nextStageOrdinal = 0;
            }
            stageToRenderToViewport = stages[nextStageOrdinal];
        }

        @Override
        public Mat processFrame(Mat input) {
            contoursList.clear();

            int yCbCrChannel = 2;
            Imgproc.cvtColor(input, yCbCr, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCr, yCbCr, yCbCrChannel);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCr, threshold, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(threshold, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(yCbCr, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contour

            for (int i = 1; i <= 3; i++) {
                double[] array;
                array = i == 1 ? leftPos : i == 2 ? midPos : rightPos;

                Imgproc.circle(yCbCr,
                        new Point((int) (cols * array[0]), (int) (rows * array[1])),
                        5,
                        new Scalar( 255, 0, 0 ),
                        1 );//draws circle

                Imgproc.rectangle(//1-3
                        yCbCr,
                        new Point(
                                cols * (array[0] - rectWidth / 2),
                                rows * (array[1] - rectHeight / 2)),
                        new Point(
                                cols * (array[0] + rectWidth / 2),
                                rows * (array[1] + rectHeight / 2)),
                        new Scalar(0, 255, 0), 3);
            }

            //get values from frame
            int lightValIndex = 0;
            double[] pixLeft = threshold.get((int) (rows * leftPos[1]), (int) (cols * leftPos[0]));//gets value at circle
            valLeft = (int) pixLeft[lightValIndex];

            double[] pixMid = threshold.get((int) (rows * midPos[1]), (int) (cols * midPos[0]));//gets value at circle
            valMid = (int) pixMid[lightValIndex];

            double[] pixRight = threshold.get((int) (rows * rightPos[1]), (int) (cols * rightPos[0]));//gets value at circle
            valRight = (int) pixRight[lightValIndex];

            switch (stageToRenderToViewport) {
                case THRESHOLD: return threshold;
                case CB: return yCbCr;
                default: return input;
            }
        }
    }
}
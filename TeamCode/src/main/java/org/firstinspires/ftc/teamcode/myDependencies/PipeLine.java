package org.firstinspires.ftc.teamcode.myDependencies;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class PipeLine extends OpenCvPipeline {

    public static int parkingPosition = 0;

    // the part of the image with the cone
    private Mat small;

    // the converted small image (rgb -> YCrCb)
    private final Mat YCrCbImage = new Mat();

    // the Cr and Cb channels of the YCrCbImage
    private final Mat RedChannel  = new Mat();
    private final Mat BlueChannel = new Mat();

    // the threshold bounded Cr and Cb channels
    private final Mat ThresholdRedImage  = new Mat();
    private final Mat ThresholdBlueImage = new Mat();

    // the part of the input image with the cone
    private final Rect coneWindowRight = new Rect(267, 128, 43, 73);
    private final Rect coneWindowLeft  = new Rect(35 , 124, 43, 73);
    private Rect coneWindow;

    // for the visual indicator
    private final Rect coneWindowOutLine;

    // the color threshold
    private final Scalar thresholdMin = new Scalar(160, 160, 160);
    private final Scalar thresholdMax = new Scalar(255, 255, 255);

    public PipeLine(Boolean isRight){
        if (isRight) {
            coneWindow = coneWindowRight;
        } else {
            coneWindow = coneWindowLeft;
        }
        coneWindowOutLine = new Rect(
                0,
                0,
                coneWindow.width,
                coneWindow.height
        );
    }

    @Override
    public Mat processFrame(Mat input){
        // get the part of the input image with the cone
        small = input.submat(coneWindow);

        // convert the small image (rgb -> YCrCb)
        Imgproc.cvtColor(small, YCrCbImage, Imgproc.COLOR_RGB2YCrCb);

        // get the Cr and Cb channels of the YCrCbImage
        Core.extractChannel(YCrCbImage, RedChannel , 1);
        Core.extractChannel(YCrCbImage, BlueChannel, 2);

        // threshold the Cr and Cb channels
        Core.inRange(RedChannel , thresholdMin, thresholdMax, ThresholdRedImage );
        Core.inRange(BlueChannel, thresholdMin, thresholdMax, ThresholdBlueImage);


        if (Core.mean(ThresholdRedImage ).val[0] > 45){// red

            parkingPosition = 2;
            // visual signal
            Imgproc.rectangle(small, coneWindowOutLine, new Scalar(255, 0  , 0  ), 2);

        } else if (Core.mean(ThresholdBlueImage).val[0] > 45){// blue

            parkingPosition = 1;
            // visual signal
            Imgproc.rectangle(small, coneWindowOutLine, new Scalar(0  , 0  , 255), 2);

        } else {// white

            parkingPosition = 0;
            // visual signal
            Imgproc.rectangle(small, coneWindowOutLine, new Scalar(255, 255, 255), 2);

        }

        // show the image
        return input;
    }
}
package org.firstinspires.ftc.teamcode.FindingEdgesCV;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

//for dashboard
/*@Config*/
public class MultiColorFilter extends OpenCvPipeline {

    //backlog of frames to average out to reduce noise
    ArrayList<double[]> frameList;
    //these are public static to be tuned in dashboard
    public static double strictLowS = 140;
    public static double strictHighS = 255;
    private ImageChannels channel;
    public MultiColorFilter() {
        frameList = new ArrayList<>();
        channel =   ImageChannels.Input_Image;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();

        //mat turns into HSV value
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        if (mat.empty()) {
            return input;
        }

        // lenient bounds will filter out near yellow, this should filter out all near yellow things(tune this if needed)
        Scalar lowHSV = new Scalar(20, 70, 80); // lenient lower bound HSV for yellow
        Scalar highHSV = new Scalar(32, 255, 255); // lenient higher bound HSV for yellow

        Scalar redlowHSV = new Scalar(0, 70, 80); // lenient lower bound HSV for red
        Scalar redhighHSV = new Scalar(10, 255, 255); // lenient higher bound HSV for red

        Scalar bluelowHSV = new Scalar(210/2, 70, 80); // lenient lower bound HSV for blue
        Scalar bluehighHSV = new Scalar(270/2, 255, 255); // lenient higher bound HSV for blue

        Mat yellowthresh = new Mat();

        // Get a black and white image of yellow objects
        Core.inRange(mat, lowHSV, highHSV, yellowthresh);

        Mat redthresh = new Mat();
        Core.inRange(mat, redlowHSV, redhighHSV, redthresh);

        Mat bluethresh = new Mat();
        Core.inRange(mat, bluelowHSV, bluehighHSV, bluethresh);

        Mat redbluethresh = new Mat();
        Core.bitwise_or(bluethresh, redthresh, redbluethresh);
        redthresh.release();
        bluethresh.release();

        Mat thresh = new Mat();
        Core.bitwise_or(redbluethresh, yellowthresh, thresh);
        redbluethresh.release();
        yellowthresh.release();

        // Mahesh try for clear:
        Mat tempthresh = new Mat();
        Mat kernel = Mat.ones(5,5, CvType.CV_32F);
        Imgproc.morphologyEx(thresh, tempthresh, Imgproc.MORPH_CLOSE, kernel);
        tempthresh.copyTo(thresh);
        tempthresh.release();
        kernel.release();
        //

        Mat masked = new Mat();
        //color the white portion of thresh in with HSV from mat
        //output into masked
        Core.bitwise_and(mat, mat, masked, thresh);
        //calculate average HSV values of the white thresh values
        Scalar average = Core.mean(masked, thresh);

        Mat scaledMask = new Mat();
        //scale the average saturation to 150
        masked.convertTo(scaledMask, -1, 150 / average.val[1], 0);


        Mat scaledThresh = new Mat();
        //you probably want to tune this
        Scalar strictLowHSV = new Scalar(0, strictLowS, 0); //strict lower bound HSV for yellow
        Scalar strictHighHSV = new Scalar(255, strictHighS, 255); //strict higher bound HSV for yellow
        //apply strict HSV filter onto scaledMask to get rid of any yellow other than pole
        Core.inRange(scaledMask, strictLowHSV, strictHighHSV, scaledThresh);

        Mat finalMask = new Mat();
        //color in scaledThresh with HSV, output into finalMask(only useful for showing result)(you can delete)
        Core.bitwise_and(mat, mat, finalMask, scaledThresh);

        Mat edges = new Mat();
        //detect edges(only useful for showing result)(you can delete)
        Imgproc.Canny(scaledThresh, edges, 100, 200);


        //Function to drawing contours
        Mat contouredImage = new Mat();
        input.copyTo(contouredImage);
        contourProcessor markContour = new contourProcessor();
        contourProcessor.markOuterContour(scaledThresh,contouredImage);

        //list of frames to reduce inconsistency, not too many so that it is still real-time, change the number from 5 if you want
        if (frameList.size() > 5) {
            frameList.remove(0);
        }

        input.release();

        // Switch to show image processing at different levels
        switch(channel) {
            case Input_Image:
               mat.copyTo(input);
               Imgproc.cvtColor(input, input, Imgproc.COLOR_HSV2RGB);
                break;
            case Color_RGB_TO_HSV:
                mat.copyTo(input);
                break;
            case Initial_Color_Mask:
                thresh.copyTo(input);
                break;
            case Merged_Mask_To_Input:
                masked.copyTo(input);
                Imgproc.cvtColor(input, input, Imgproc.COLOR_HSV2RGB);
                break;
            case Scaled_Mask_To_FullColor:
                scaledThresh.copyTo(input);
                break;
            case Fully_Masked_Image:
                finalMask.copyTo(input);
                Imgproc.cvtColor(input, input, Imgproc.COLOR_HSV2RGB);
                break;
            case Show_Contour_Edges:
                edges.copyTo(input);
                break;
            case Show_Simplified_Edge:
                contouredImage.copyTo(input);
                break;
        }

        //release all the data
        scaledThresh.release();
        scaledMask.release();
        mat.release();
        masked.release();
        edges.release();
        thresh.release();
        finalMask.release();
        contouredImage.release();

        return input;
    }

    public void putImageChannel(ImageChannels channel1){
        channel = channel1;
     }
    public ImageChannels getImageChannel(){
        return channel;
    }

}
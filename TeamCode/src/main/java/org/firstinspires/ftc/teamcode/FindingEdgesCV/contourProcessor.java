package org.firstinspires.ftc.teamcode.FindingEdgesCV;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class contourProcessor {
    public void CvMath(){}

    /**
     * Used to mark outer rectangle and its corners.
     *
     * @param processedImage Image used for calculation of contours and corners.
     * @param originalImage Image on which marking is done.
     */
    public static void markOuterContour(final Mat processedImage,
                                        final Mat originalImage) {
        // Find contours of an image
        final List<MatOfPoint> allContours = new ArrayList<>();
        Imgproc.findContours(
                processedImage,
                allContours,
                new Mat(processedImage.size(), processedImage.type()),
                Imgproc.RETR_TREE,
                Imgproc.CHAIN_APPROX_NONE
        );

        // Filter out noise and display contour area value
        final List<MatOfPoint> filteredContours = allContours.stream()
                .filter(contour -> {
                    final double value = Imgproc.contourArea(contour);
                    final boolean isNotNoise = value > 1000;
                    return isNotNoise;
                }).collect(Collectors.toList());

        // Mark contours


        filteredContours.stream()
                .forEach(contour ->{
                    MatOfPoint2f dst = new MatOfPoint2f();
                    MatOfPoint approxf1 = new MatOfPoint();

                    contour.convertTo(dst, CvType.CV_32F);
                    Imgproc.approxPolyDP(dst, dst, 0.01 * Imgproc.arcLength(dst, true), true);
                    dst.convertTo(approxf1, CvType.CV_32S);
                    List<MatOfPoint> contourTemp = new ArrayList<>();
                    contourTemp.add(approxf1);
                    Imgproc.drawContours(
                            originalImage,
                            contourTemp,
                            -1, // Negative value indicates that we want to draw all of contours
                            new Scalar(124, 252, 0), // Green color
                            1
                    );

                    final Rect rect = Imgproc.boundingRect(contour);
                    String shape = "??";
                    if (dst.toArray().length==3){shape = "Triangle";}
                    else if (dst.toArray().length==4){shape="Quadrilaterals";}
                    else if (dst.toArray().length>10){shape = "May be Circle";}

                    Imgproc.putText (
                            originalImage,
                            "Shape: " + shape,
                            new Point(rect.x + rect.width, rect.y + rect.height + 15),
                            2,
                            0.5,
                            new Scalar(124, 252, 0),
                            1
                    );
                }
        );

    }
    // endregion
    public static double getLineAngle(Point pL,Point pIntersect,Point pR) {

        double angle1 = Math.atan2(pL.y - pIntersect.y, pL.x - pIntersect.x);
        double angle2 = Math.atan2(pR.y - pIntersect.y, pR.x - pIntersect.x);

        return angle1 - angle2;
    }
    public static double getDistance (Point pL,Point pR){
        return Math.sqrt(Math.pow((pL.x-pR.x), 2) + Math.pow((pL.y-pR.y), 2));
    }
}

package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;


import com.qualcomm.ftcrobotcontroller.opmodes.cheer4ftc.LinearOpModeCamera;

import org.bytedeco.javacpp.opencv_core.Mat;
import org.bytedeco.javacpp.opencv_objdetect.

/**
 * Created by nathaniel on 10/3/15.
 */
public class OpenCVTest1 extends LinearOpModeCamera {

    @Override
    public void runOpMode() throws InterruptedException {
        this.yuvImage
    }

    public void detectEdges() {
        // init
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        // find contours
        Imgproc.findContours(maskedImage, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

        // if any contour exist...
        if (hierarchy.size().height > 0 && hierarchy.size().width > 0)
        {
            // for each contour, display it in blue
            for (int idx = 0; idx >= 0; idx = (int) hierarchy.get(0, idx)[0])
            {
                Imgproc.drawContours(frame, contours, idx, new Scalar(250, 0, 0));
            }
        }
    }
}

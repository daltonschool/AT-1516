package com.qualcomm.ftcrobotcontroller.opmodes.cheer4ftc;

import android.graphics.Bitmap;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */
public class DetectColor extends LinearOpModeCamera {
  int ds2 = 1;  // additional downsampling of the image
  // set to 1 to disable further downsampling

  @Override
  public void runOpMode() throws InterruptedException {

    String colorString = "NONE";
    String colorString2 = "NONE";
    int blueValue1 = 0;
    int blueValue2 = 0;
    int redValue1 = 0;
    int redValue2 = 0;
    // linear OpMode, so could do stuff like this too.
        /*
        motorLeft = hardwareMap.dcMotor.get("motor_1");
        motorRight = hardwareMap.dcMotor.get("motor_2");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        */

    if (isCameraAvailable()) {

      setCameraDownsampling(8);
      // parameter determines how downsampled you want your images
      // 8, 4, 2, or 1.
      // higher number is more downsampled, so less resolution but faster
      // 1 is original resolution, which is detailed but slow
      // must be called before super.init sets up the camera

      startCamera();  // can take a while.
      // best started before waitForStart
      // or in a separate thread.

      waitForStart();

      stopCameraInSecs(30);   // set independent thread to kill the camera
      // when the mode is done
      // use 30 for auto, 120 for teleop

      // LinearOpMode, so could do stuff like this too.
        /*
        motorLeft.setPower(1);  // drive forward
        motorRight.setPower(1);
        sleep(1000);            // for a second.
        motorLeft.setPower(0);  // stop drive motors.
        motorRight.setPower(0);
        sleep(1000);            // wait a second.
        */

      while (opModeIsActive()) {
        if (imageReady()) { // only do this if an image has been returned from the camera
          int redValue = 0;
          int blueValue = 0;
          int greenValue = 0;

          Bitmap rgbImage;
          rgbImage = convertYuvImageToRgb(yuvImage, width, height, ds2);
          for (int x = 0; x < width/2; x++) {
            for (int y = 0; y < height; y++) {
              int pixel = rgbImage.getPixel(x, y);
              redValue1 += red(pixel);
              blueValue1 += blue(pixel);
              greenValue += green(pixel);
            }
          }

          redValue = blueValue = greenValue = 0;

          for (int x = width/2; x < width; x++) {
            for (int y = 0; y < height; y++) {
              int pixel = rgbImage.getPixel(x, y);
              redValue2 += red(pixel);
              blueValue2 += blue(pixel);
              greenValue += green(pixel);
            }
          }


        } else {
          blueValue1 = blueValue2 = redValue1 = redValue2 = 0;
        }

        telemetry.addData("Left:", blueValue1 > redValue1 ? "Blue" : "Red");
        telemetry.addData("Right:", blueValue2 > redValue2 ? "Blue" : "Red");
        telemetry.addData("Bluer side:", blueValue1 > blueValue2 ? "LEFT" : "RIGHT");

        blueValue1 = blueValue2 = redValue1 = redValue2 = 0;
        waitOneFullHardwareCycle();
      }
    }
  }
}
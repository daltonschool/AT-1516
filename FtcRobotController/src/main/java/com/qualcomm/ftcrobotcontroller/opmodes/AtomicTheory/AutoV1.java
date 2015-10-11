package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Random;

/**
 * Created by davis on 10/6/15.
 *
 * @author Davis Haupt
 *
 * Abstract autonomous OpMode.
 *
 * Basically, this OpMode starts some distance away from the beacon,
 * detects which side of the beacon corresponds to which color,
 * approaches the beacon, and pushes the correct button.
 *
 * It can also place the climbers in the bin.
 *
 * Concrete sub-classes of this class implement the
 * getTeam() method, so that there are separate OpModes for the
 * Red and Blue alliances.
 */
public abstract class AutoV1 extends AtomicBaseLinearOpMode {
  int ds2 = 1;  // additional downsampling of the image

  public void runOpMode() throws InterruptedException{
    config();
    if (isCameraAvailable()) {
      setCameraDownsampling(8);
      startCamera();

      waitForStart();
      stopCameraInSecs(60);

      driveTicks(0, 50, Direction.FORWARD); // Go forth unafraid
      rotateTicks(0, 25, Direction.COUNTERCLOCKWISE); // rotate 90 deg to the left
      driveTicks(0, 25, Direction.FORWARD); // allign with the beacon
      rotateTicks(0, 25, Direction.COUNTERCLOCKWISE); // rotate towards the beacon

      // take a picture, and analyze it for the beacon colors.

      Bitmap rgbImage = convertYuvImageToRgb(yuvImage, width, height, ds2);
      int[][] rgbLevels = colorLevels(rgbImage, 2);
      Alliance[] beacon = findBeaconColors(rgbLevels);
      Direction pushDir = getPush(beacon[0], beacon[1]);
      pushButton(pushDir); // extend the correct side of the bopper
      // move towards the beacon, in the process, hitting the button.
      driveTicks(0, 25, Direction.FORWARD);
      dumpClimbers();
    }
  }

  public void dumpClimbers() {
    // CR servos are fucked up
  }

  public void pushButton(Direction p) {
    switch(p) {
      case LEFT:
        bopper.setPosition(0.0);
        break;
      case RIGHT:
        bopper.setPosition(1.0);
        break;
      default:
        bopper.setPosition(0.5);
        break;
    }
  }

  public Alliance[] findBeaconColors(int[][] rgbLevels) {
    Alliance[] a = new Alliance[2];
    // Assign RED or BLUE to each side of the beacon depending on which
    // color is most prevalent in that side of the image.
    a[0] = rgbLevels[0][0] > rgbLevels[0][2] ? Alliance.RED : Alliance.BLUE;
    a[1] = rgbLevels[1][0] > rgbLevels[1][2] ? Alliance.RED : Alliance.BLUE;

    // If both sides are the same color,
    // determine which side is *more* blue, and set that as BLUE.
    if (a[0] == a[1]) {
      if (rgbLevels[0][2] > rgbLevels[1][2]) {
        a[0] = Alliance.BLUE;
        a[1] = Alliance.RED;
      }
      else if (rgbLevels[0][2] < rgbLevels[1][2]) {
        a[0] = Alliance.RED;
        a[1] = Alliance.BLUE;
      }
      else { // if the values are exactly the same, set both to null.
        a[0] = a[1] = null;
      }
    }

    return a;
  }

  /**
   *  if the robot is on the same alliance as the left button, return LEFT
   *  otherwise, i.e. the robot is on the opposite as the left button, return RIGHT.
   * @param l color of left side of the beacon
   * @param r color of right side of the beacon
   * @return direction to push
   */
  public Direction getPush(Alliance l, Alliance r) {
    if (getTeam() == l) return Direction.LEFT;
    else return Direction.RIGHT;
  }

  /**
   *
   * @param img the bitmap image to process
   * @param k the number of vertical sections the image should be sliced into
   * @return 2d array with the total R,G,B values (respectively) in each subinterval
   * within the image. so vals[1][2] would be the green value from the 2nd
   * sections.
   */
  private int[][] colorLevels(Bitmap img, int k) {
    int[][] vals = new int[k][3];

    for (int i = 0; i < vals.length; i++) {
      for (int x = 0; x < width/k * (i+1); x++) {
        for (int y = 0; y < height; y++) {
          int pixel = img.getPixel(x, y);
          vals[i][0] = red(pixel);
          vals[i][1] = green(pixel);
          vals[i][2] = blue(pixel);
        }
      }
    }

    return vals;
  }
}

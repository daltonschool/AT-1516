package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

import android.graphics.Bitmap;
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
      stopCameraInSecs(30);

      Bitmap rgbImage = convertYuvImageToRgb(yuvImage, width, height, ds2);

      int[][] pic = colorLevels(rgbImage, 2);

      // Assign RED or BLUE to each side of the beacon depending on which
      // color is most prevalent in that side of the image.
      Alliance leftButton = pic[0][0] > pic[0][2] ? Alliance.RED : Alliance.BLUE;
      Alliance rightButton = pic[1][0] > pic[1][2] ? Alliance.RED : Alliance.BLUE;

      // If both sides are the same color,
      // determine which side is *more* blue, and set that as BLUE.
      if (leftButton == rightButton) {
        if (pic[0][2] > pic[1][2]) {
          leftButton = Alliance.BLUE;
          rightButton = Alliance.RED;
        } else if(pic[0][2] > pic[1][2]) {
          leftButton = Alliance.RED;
          rightButton = Alliance.BLUE;
        } else; //we can't figure it out so do nothing. not worth it to give the other team points.
      }
    }
  }

  /**
   *  if the robot is on the same side as the left button, return LEFT
   *  otherwise, i.e. the robot is on the opposite as the left button, return RIGHT.
   * @param l color of left side of the beacon
   * @param r color of right side of the beacon
   * @return side to push
   */
  public int getPush(Alliance l, Alliance r) {
    if (getTeam() == l) return LEFT;
    else return RIGHT;
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

    for (int i = 0; i < k; i++) {
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

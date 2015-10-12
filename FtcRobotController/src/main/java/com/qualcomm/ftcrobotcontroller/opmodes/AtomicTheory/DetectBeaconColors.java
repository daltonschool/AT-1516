package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

import android.graphics.Bitmap;
import com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory.AtomicUtil.*;

/**
 * Created by davis on 10/12/15.
 */
public class DetectBeaconColors extends AtomicBaseOpMode{
  int ds2 = 1;

  public void loop() {
    Bitmap rgbImage = convertYuvImageToRgb(yuvImage, width, height, ds2);

    int[][] pics = colorLevels(rgbImage, 2);
    Alliance[] beacon = findBeaconColors(pics);

    switch(beacon[0]) {
      case RED:
        telemetry.addData("Left", "RED");
        break;
      case BLUE:
        telemetry.addData("Left", "BLUE");
        break;
    }
    switch(beacon[1]) {
      case RED:
        telemetry.addData("Right", "RED");
        break;
      case BLUE:
        telemetry.addData("Right", "BLUE");
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

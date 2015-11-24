package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

/**
 * Created by davis on 11/21/15.
 */
public class AutoV3 extends LinearAlpha{
  public void runOpMode() throws InterruptedException{
    setup();
    waitForStart();
    drive(-0.70);
    long t = System.currentTimeMillis();
    while (System.currentTimeMillis() - t < 5000)
      ;
    stopMotors();
  }
}

package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

/**
 * Created by davis on 11/21/15.
 */
public class AutoRed extends LinearAlpha{
  public void runOpMode() throws InterruptedException{
    setup();
    waitForStart();
    driveTicks(0.5, 4165);
    sleep(500);
    rotateTicks(.8, AtomicUtil.Direction.COUNTERCLOCKWISE, 650);
    sleep(500);
    driveTicks(0.5, 4500);
    sleep(500);
    driveTicks(-0.5, 731);
    sleep(500);
    rotateTicks(.8, AtomicUtil.Direction.COUNTERCLOCKWISE, 510);
    sleep(500);
    driveTicks(.5, 500);
  }
}

package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

/**
 * Created by davis on 10/15/15.
 */
public class FuckThisShit extends AtomicBaseLinearOpMode{
  public void runOpMode() throws InterruptedException {
    config();
    waitForStart();
    lift.setPosition(1);
    long t = System.currentTimeMillis();
    while (System.currentTimeMillis() - t < 5000)
      ;

    lift.setPosition(.493);
    drop.setPosition(0);
    bopper.setPosition(0);
  }
}

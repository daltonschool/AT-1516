package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

/**
 * Created by davis on 12/28/15.
 */
public class ForwardTest extends LinearAlpha{
  public void runOpMode() throws InterruptedException{
    setup();
    waitForStart();
    driveTicks(50, 5000);
  }
  AtomicUtil.Alliance getTeam() {
    return null;
  }
}

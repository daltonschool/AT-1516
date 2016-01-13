package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

/**
 * Created by davis on 1/12/16.
 */
public class RotateTest extends LinearAlpha{
  AtomicUtil.Direction turnDir;
  int turnK = 0;
  AtomicUtil.Alliance getTeam() {
    return AtomicUtil.Alliance.RED;
  }

  public void runOpMode() throws InterruptedException {
    setup();
    switch (getTeam()) {
      case BLUE:
        turnDir = AtomicUtil.Direction.CLOCKWISE;
        turnK = -1;
        break;
      case RED:
        turnDir = AtomicUtil.Direction.COUNTERCLOCKWISE;
        turnK = 1;
        break;
    }

    waitForStart();
    double startHeading = curHeading+180;
    double turn = 85.0;
    double targetHeading = (startHeading + turn*turnK)%360;
    while(curHeading+180 < targetHeading) {
      rotate(.8, turnDir);
    }
    stopMotors();
    telemetry.addData("Right", right.getPower());
    telemetry.addData("Left", left.getPower());
  }
}

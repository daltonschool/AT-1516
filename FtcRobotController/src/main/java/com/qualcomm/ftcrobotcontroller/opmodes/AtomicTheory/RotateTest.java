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
        turnK = 1;
        break;
      case RED:
        turnDir = AtomicUtil.Direction.COUNTERCLOCKWISE;
        turnK = -1;
        break;
    }

    updateHeading();
    double startHeading = curHeading+180;
    double targetHeading = (startHeading + 90)%360;
//    double error;
//    do {
//      updateHeading();
//
//      error = curHeading+180 - targetHeading;
//      double error_const = .04;
//      double pl = scale(- error*error_const);
//      double pr = scale(error*error_const);
//
//      moveLeft(pl);
//      moveRight(pr);
//    } while (Math.abs(error) > 1);
    while(curHeading+180 < targetHeading) {
      updateHeading();
      rotate(.8, turnDir);
    }
    stopMotors();
    left.setPower(0);
    right.setPower(0);
    sleep(10);
    telemetry.addData("Right", right.getPower());
    telemetry.addData("Left", left.getPower());
  }
}

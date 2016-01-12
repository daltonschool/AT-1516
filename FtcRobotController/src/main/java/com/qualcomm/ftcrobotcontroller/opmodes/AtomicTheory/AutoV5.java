package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

/**
 * Created by davis on 1/11/16.
 */
public abstract class AutoV5 extends LinearAlpha{
  AtomicUtil.Direction turnDir;
  int turnK = 0;
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
    waitForStart();
    driveTicksStraight(0.5, 4200);
    sleep(500);
    rotateTicks(.8, turnDir, 870);
    sleep(500);
    driveTicksStraight(0.5, 3700);

//    updateHeading();
//    double startHeading = curHeading+180;
//    double targetHeading = (startHeading + 90)%360;
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
//    stopMotors();
//    left.setPower(0);
//    right.setPower(0);
//    sleep(10);
//    telemetry.addData("Right", right.getPower());
//    telemetry.addData("Left", left.getPower());
  }
}

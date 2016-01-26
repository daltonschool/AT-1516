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
        turnK = -1;
        break;
      case RED:
        turnDir = AtomicUtil.Direction.COUNTERCLOCKWISE;
        turnK = 1;
        break;
    }
    waitForStart();
    driveTicksStraight(0.5, 8900);
    sleep(500);
    rotateTicks(.8, turnDir, 410);
    sleep(500);
    driveTicksStraight(0.5, 1000);
    sleep(500);
//    dump.setPosition(0);

//    double startHeading = curHeading+180;
//    double targetHeading = (startHeading + 40*turnK)%360;
//    while(curHeading+180 < targetHeading) {
//      rotate(.8, turnDir);
//    }
//    stopMotors();
//    sleep(500);
//    driveTicksStraight(0.5, 3700);
  }
}

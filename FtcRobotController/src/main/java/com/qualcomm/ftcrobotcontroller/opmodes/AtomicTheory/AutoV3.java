package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

/**
 * Created by davis on 12/14/15.
 */
import com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory.AtomicUtil.*;
public abstract class AutoV3 extends LinearAlpha{
  Direction turnDir;
  public void runOpMode() throws InterruptedException{
    setup();
    switch(getTeam()) {
      case BLUE:
        turnDir = Direction.CLOCKWISE;
        break;
      case RED:
        turnDir = Direction.COUNTERCLOCKWISE;
        break;
    }
    waitForStart();
    dumbTicks(0.5, 4165);
    sleep(500);
    rotateTicks(.8, turnDir, 650);
    sleep(500);
    dumbTicks(0.5, 4500);
    sleep(500);
    dumbTicks(-0.5, 731);
    sleep(500);
    rotateTicks(.8, turnDir, 510);
    sleep(500);
    dumbTicks(.5, 500);
  }
}

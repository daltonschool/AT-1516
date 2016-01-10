package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

/**
 * Created by nathaniel on 1/9/16
 * note that once this is the main autonomous we will have to update
 * AutoRed and AutoBlue classes
 */

import com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory.AtomicUtil.*;

public abstract class AutoV4 extends LinearAlpha{


    Direction turnDir;
    int turnK = 0;
    public void runOpMode() throws InterruptedException{
        setup();
        switch(getTeam()) {
            case BLUE:
                turnDir = Direction.CLOCKWISE;
                turnK = 1;
                break;
            case RED:
                turnDir = Direction.COUNTERCLOCKWISE;
                turnK = -1;
                break;
        }
        waitForStart();
        driveTicksStraight(0.5, 4165);
        sleep(500);
        rotateTicks(.8, turnDir, 650);
//        rotateDegs(.8*turnK, 45);
        sleep(500);
        driveTicksStraight(0.5, 4500);
        sleep(500);
        driveTicksStraight(-0.5, 731);
        sleep(500);
        rotateTicks(.8, turnDir, 510);
//        rotateDegs(.8*turnK, 45);
        sleep(500);
        driveTicksStraight(.5, 500);
    }
}

package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

/**
 * Created by nathaniel on 1/9/16
 * note that once this is the main autonomous we will have to update
 * AutoRed and AutoBlue classes
 */

import com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory.AtomicUtil.*;

public abstract class AutoV4 extends LinearAlpha{


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

        turnToHeading(90);

    }
}

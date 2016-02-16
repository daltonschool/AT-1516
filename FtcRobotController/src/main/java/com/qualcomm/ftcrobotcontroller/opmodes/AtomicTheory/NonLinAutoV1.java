package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

import com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory.AtomicUtil.*;

/**
 * Created by nathaniel on 2/2/16.
 */
public abstract class NonLinAutoV1 extends BaseAutoNonLin {

    final int final_state = 1001;

    Direction dir;
    double drivePower = .6;


    public void init() {
        dir = (getTeam() == Alliance.RED) ? Direction.COUNTERCLOCKWISE : Direction.CLOCKWISE;
    }

    public void loop() {

        switch(state) {
            case 0: {
                updateGlobalHeadingAndEncodeInformation();

                int tickstoDrive = 500; //obviously this values aren't proven at all

                driveTicksStraight(drivePower, tickstoDrive); //updates state automagically

                state = final_state;

                break;
            }

            case 1: {
                updateGlobalHeadingAndEncodeInformation();

                int degrees = 60; //update these values

                rotateDegs(drivePower, dir, degrees);

                break;
            }

            case 2: {

            }

            case final_state: {
                stopMotors();
                break;
            }
        }
    }

}

package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

import com.qualcomm.robotcore.robocol.Telemetry;

/**
 * Created by nathaniel on 10/12/15.
 */

/*

my rough draft for an autonomous

note the encoderZero variable.
 */

public class AutoMoveTesting extends AtomicBaseOpMode {

    int state = 0;
    int encoderZero = 0;

    @Override
    public void loop() {
        switch(state) {
            case 0: {
                //drive forwards
                //blargh...
                if(Math.abs(BR.getCurrentPosition() - encoderZero) < 2000) {
                    BL.setPower(-0.5);
                    BR.setPower(-0.5);
                    FL.setPower(-0.5);
                    FR.setPower(-0.5);
                } else {
                    BL.setPower(0);
                    BR.setPower(0);
                    FL.setPower(0);
                    FR.setPower(0);

                    encoderZero = BL.getCurrentPosition();

                    state=1;
                }
            }
            case 1: {
                //turn left
                if(Math.abs(BR.getCurrentPosition() - encoderZero) < 1800) {
                    BL.setPower(0.5);
                    BR.setPower(-0.5);
                    FL.setPower(0.5);
                    FR.setPower(-0.5);
                } else {
                    BL.setPower(0);
                    BR.setPower(0);
                    FL.setPower(0);
                    FR.setPower(0);

                    encoderZero = BL.getCurrentPosition();

                    state=2;
                }

            }
            case 2: {
                //drive forwards


            }
            case 3:
        }
    }
}

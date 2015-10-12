package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

import com.qualcomm.robotcore.robocol.Telemetry;

/**
 * Created by nathaniel on 10/12/15.
 */

/*not used anymore :D
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
                if(BL.getCurrentPosition() - encoderZero < 1000) {
                    BL.setPower(.0);
                    BR.setPower(1.0);
                    FL.setPower(1.0);
                    FR.setPower(1.0);
                } else {
                    BL.setPower(0);
                    BR.setPower(0);
                    FL.setPower(0);
                    FR.setPower(0);

                    encoderZero = BL.getCurrentPosition();

                    state++;
                }
            }
            case 1: {
                //turn left


            }
            case 2: {
                //drive forwards


            }
            case 3:
        }
    }
}

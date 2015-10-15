package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

import android.graphics.Bitmap;

import org.apache.http.impl.conn.DefaultHttpRoutePlanner;

/**
 * Created by nathaniel on 10/13/15.
 */
public class TestingServos extends AtomicBaseOpMode {

    int state = 0;
    long sleepUntil = 0;

    int ds2 = 1;

    @Override
    public void loop() {
        telemetry.addData("state: ", Integer.toString(state));

        if (sleepUntil > System.nanoTime()) {

        } else if (state == 0) {
            drop.setPosition(0.1);
            sleep(3000);
            state=1;
        } else if(state == 1) {
            drop.setPosition(1.0);
        }

    }


    public void sleep(int ms) {
        int ns = ms * 1000000000;
        sleepUntil = System.nanoTime()+ns;
    }


}

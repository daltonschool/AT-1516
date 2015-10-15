package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

import com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory.AtomicUtil.Alliance;
import com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory.AtomicUtil.Direction;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robocol.Telemetry;

/**
 * Created by nathaniel on 10/12/15.
 */

/*

my rough draft for an autonomous

note the encoderZero variable.
 */

public class AutoMoveTesting extends AtomicBaseOpMode {

    Alliance[] beacon;
    Direction pushDir;
    Bitmap rgbImage;

    int state = 0;
    int encoderZero = 0;
    long sleepUntil = 0;

    int ds2 = 1;

    @Override
    public void loop() {
        telemetry.addData("state: ", Integer.toString(state));

        if(sleepUntil > System.nanoTime());
        else if (state == 0) {
            //drive forwards
            //blargh...
            if (Math.abs(BR.getCurrentPosition() - encoderZero) < 3000) {
                BL.setPower(-0.25);
                BR.setPower(-0.25);
                FL.setPower(-0.25);
                FR.setPower(-0.25);
            } else {
                BL.setPower(0);
                BR.setPower(0);
                FL.setPower(0);
                FR.setPower(0);

                encoderZero = BR.getCurrentPosition();

                state = 1;
                sleep(1000);
            }

        } else if(state == 1) {
            //turn left 90 deg
            if(Math.abs(BR.getCurrentPosition() - encoderZero) < 1600) {
                BL.setPower(0.25);
                BR.setPower(-0.25);
                FL.setPower(0.25);
                FR.setPower(-0.25);
            } else {
                BL.setPower(0);
                BR.setPower(0);
                FL.setPower(0);
                FR.setPower(0);

                encoderZero = BR.getCurrentPosition();

                state=2;
                sleep(1000);
            }
        } else if(state == 2) {
            //go forwards
            if(Math.abs(BR.getCurrentPosition() - encoderZero) < 3500) {
                BL.setPower(-0.25);
                BR.setPower(-0.25);
                FL.setPower(-0.25);
                FR.setPower(-0.25);
            } else {
                BL.setPower(0);
                BR.setPower(0);
                FL.setPower(0);
                FR.setPower(0);

                encoderZero = BR.getCurrentPosition();

                state=3;
                sleep(1000);
            }
        } else if(state == 3) {
            //turn left 90 deg
            if (Math.abs(BR.getCurrentPosition() - encoderZero) < 1800) {
                BL.setPower(0.25);
                BR.setPower(-0.25);
                FL.setPower(0.25);
                FR.setPower(-0.25);
            } else {
                BL.setPower(0);
                BR.setPower(0);
                FL.setPower(0);
                FR.setPower(0);

                encoderZero = BR.getCurrentPosition();

                state = 4;
                sleep(1000);
            }
        } else if(state == 4) {
            //go forwards to beacon
            if (Math.abs(BR.getCurrentPosition() - encoderZero) < 500) { //not used right now
                BL.setPower(-0.25);
                BR.setPower(-0.25);
                FL.setPower(-0.25);
                FR.setPower(-0.25);
            } else {
                BL.setPower(0);
                BR.setPower(0);
                FL.setPower(0);
                FR.setPower(0);

                encoderZero = BR.getCurrentPosition();

                state = 5; //switch back to 5 when we figure out what's wrong with camera.
                sleep(1000);
            }
        } else if(state == 5) {
            //take a picture
            rgbImage = convertYuvImageToRgb(yuvImage, width, height, ds2);
            sleep(3000); //sleep just in case this is critical #BringHimHome //no idea how long to wait
            state = 6;
        } else if(state == 6) {
            //process image
            int[][] rgbLevels = colorLevels(rgbImage, 2);
            Alliance[] beacon = findBeaconColors(rgbLevels);
            pushDir = getPush(beacon[0], beacon[1]);
            //shouldn't require sleeping because time to process is contained
            sleep(2000);
            state = 7;
        } else if(state == 7) { //shitty insert I'm so sorry
            lift.setPosition(1);
            sleep(2000);
            state = 8;
        } if(state == 8) {
            //push the button
            lift.setPosition(.493);
            pushButton(pushDir); // extend the correct side of the bopper
            sleep(3000);
            state=9;
        } else if(state == 9) {
            //drive forwards to hit button
            if (Math.abs(BR.getCurrentPosition() - encoderZero) < 1000) {
                BL.setPower(-0.25);
                BR.setPower(-0.25);
                FL.setPower(-0.25);
                FR.setPower(-0.25);
            } else {
                BL.setPower(0);
                BR.setPower(0);
                FL.setPower(0);
                FR.setPower(0);

                encoderZero = BR.getCurrentPosition();

                state = 10;
                sleep(500);
            }
        } else if(state == 10) { //drop the climbers
            drop.setPosition(0.0);
            sleep(2500);
            if(drop.getPosition() != 0.0);
            else
               state = 11;
        } else if(state == 11) {
            drop.setPosition(1.0); //replace drop
            sleep(2500);
            if(drop.getPosition() != 1.0);
            else
                state = 12;
        } else if(state == 12) { //move back to prepare to go up ramp
            if(Math.abs(BR.getCurrentPosition() - encoderZero) < 6400) {
                BL.setPower(0.25);
                BR.setPower(0.25);
                FL.setPower(0.25);
                FR.setPower(0.25);
            } else {
                BL.setPower(0);
                BR.setPower(0);
                FL.setPower(0);
                FR.setPower(0);

                encoderZero = BR.getCurrentPosition();

                state=13;
                sleep(500);
            }
        } else if(state == 13) {
            //turn 45 deg counter clockwise to go up ramp
            if(Math.abs(BR.getCurrentPosition() - encoderZero) < 600) {
                BL.setPower(-0.25);
                BR.setPower(0.25);
                FL.setPower(-0.25);
                FR.setPower(0.25);
            } else {
                BL.setPower(0);
                BR.setPower(0);
                FL.setPower(0);
                FR.setPower(0);

                encoderZero = BR.getCurrentPosition();

                state=14;
                sleep(500);
            }
        } else if(state == 14) {
            //attack the ramp
            if(Math.abs(BR.getCurrentPosition() - encoderZero) < 10000) {
                BL.setPower(-1);
                BR.setPower(-1);
                FL.setPower(-1);
                FR.setPower(-1);
            } else {
                BL.setPower(0);
                BR.setPower(0);
                FL.setPower(0);
                FR.setPower(0);

                encoderZero = BR.getCurrentPosition();

                state=15;
                sleep(500);
            }
        }
    }


    public void sleep(int ms) {
        int ns = ms * 1000000000;
        sleepUntil = System.nanoTime()+ns;
    }

    public Alliance[] findBeaconColors(int[][] rgbLevels) {
        Alliance[] a = new Alliance[2];
        // Assign RED or BLUE to each side of the beacon depending on which
        // color is most prevalent in that side of the image.
        a[0] = rgbLevels[0][0] > rgbLevels[0][2] ? Alliance.RED : Alliance.BLUE;
        a[1] = rgbLevels[1][0] > rgbLevels[1][2] ? Alliance.RED : Alliance.BLUE;

        // If both sides are the same color,
        // determine which side is *more* blue, and set that as BLUE.
        if (a[0] == a[1]) {
            if (rgbLevels[0][2] > rgbLevels[1][2]) {
                a[0] = Alliance.BLUE;
                a[1] = Alliance.RED;
            }
            else if (rgbLevels[0][2] < rgbLevels[1][2]) {
                a[0] = Alliance.RED;
                a[1] = Alliance.BLUE;
            }
            else { // if the values are exactly the same, set both to null.
                a[0] = a[1] = null;
            }
        }

        return a;
    }

    private int[][] colorLevels(Bitmap img, int k) {
        int[][] vals = new int[k][3];

        for (int i = 0; i < vals.length; i++) {
            for (int x = 0; x < width/k * (i+1); x++) {
                for (int y = 0; y < height; y++) {
                    int pixel = img.getPixel(x, y);
                    vals[i][0] = red(pixel);
                    vals[i][1] = green(pixel);
                    vals[i][2] = blue(pixel);
                }
            }
        }

        return vals;
    }

    public Direction getPush(Alliance l, Alliance r) {
        if (getTeam() == l) return Direction.LEFT;
        else return Direction.RIGHT;
    }

    public void pushButton(Direction p) {
        switch(p) {
            case LEFT:
                bopper.setPosition(0.0);
                break;
            case RIGHT:
                bopper.setPosition(1.0);
                break;
            default:
                bopper.setPosition(0.5);
                break;
        }
    }

    private Alliance getTeam() {
        return Alliance.BLUE;
    }
}

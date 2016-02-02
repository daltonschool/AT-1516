package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

import com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory.Archived.AlphaDrive;

/**
 * Created by davis on 10/10/15.
 */
public class FpsDrive extends AlphaDrive {
  public void loop() {
    float throttle = gamepad1.left_stick_y;
    float turn = gamepad1.right_stick_x;

    double leftPower = scale_motor_power(throttle - turn);
    double rightPower = scale_motor_power(throttle + turn);

    moveLeft(leftPower);
    moveRight(rightPower);

    if (gamepad1.a)
      aimUp();
    else if (gamepad1.b)
      aimDown();
    else if (Math.abs(gamepad2.right_stick_y) > .05)
      smoothAim(gamepad2.right_stick_y);
    else
      noAB();

    if (gamepad1.x)
      pressX();
    else if (gamepad1.y)
      pressY();
    else if (Math.abs(gamepad2.left_stick_y) > .05)
      pullUp(scale_motor_power(gamepad2.left_stick_y));
    else
      noXY();

    if (gamepad2.a)
      syncAim(AIM_PRESET_INTO_THIN_AIR);
//    else if(gamepad2.b) {
//      syncAim(AIM_PRESET_MID_ZONE);
//    }

    if (gamepad1.left_bumper)
      dumpDown();
    else if (gamepad1.right_bumper)
      dumpUp();

    if (gamepad1.dpad_up)
      resetZips();
    else if (gamepad1.dpad_left)
      leftZipDown();
    else if (gamepad1.dpad_right)
      rightZipDown();
    else if (gamepad2.dpad_up)
      resetZips();
    else if (gamepad2.dpad_left)
      leftZipDown();
    else if (gamepad2.dpad_right)
      rightZipDown();

    if(gamepad2.left_bumper)
      dumpDown();
    else if (gamepad2.right_bumper)
      dumpUp();

//    if (gamepad2.b) {
//      rightZipDown();
//    }
//    if (gamepad2.x) {
//      leftZipDown();
//    }
//    if (gamepad2.y) {
//      resetZips();
//    }

//    if (gamepad2.left_trigger > .1)
//      pressLT(gamepad2.left_trigger);
//    else if(gamepad2.right_trigger > .1)
//      pressRT(gamepad2.right_trigger);
//    else
//      noPressT();


    writeTelemetry();
  }
}

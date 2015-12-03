package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

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
      pressA();
    else if (gamepad1.b)
      pressB();
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
      syncAim();

    if (gamepad1.left_bumper)
      moveLift(1);
    else if (gamepad1.right_bumper)
      moveLift(-1);
    else
      moveLift(0);

    if (gamepad1.dpad_up)
      releaseZipline();
    else if (gamepad1.dpad_left)
      engageZipline(AtomicUtil.Direction.LEFT);
    else if (gamepad1.dpad_right)
      engageZipline(AtomicUtil.Direction.RIGHT);

    if(gamepad2.left_bumper)
      pressLB();
    else if (gamepad2.right_bumper)
      pressRB();

    if (gamepad2.left_trigger > .1)
      pressLT(gamepad2.left_trigger);
    else if(gamepad2.right_trigger > .1)
      pressRT(gamepad2.right_trigger);
    else
      noPressT();


    writeTelemetry();
  }
}

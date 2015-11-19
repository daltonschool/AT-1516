package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

/**
 * Created by davis on 11/18/15.
 */
public class SingleDrive extends AlphaDrive{
  public void loop() {
    double throttle;
    if (gamepad1.right_bumper) // speed
      throttle = 1.0;
    else if (gamepad1.left_bumper) // precision
      throttle = 0.4;
    else
      throttle = 0.6; // default

    if (gamepad1.dpad_up)
      drive(throttle);
    else if (gamepad1.dpad_down)
      drive(-throttle);
    else if (gamepad1.dpad_left)
      rotate(throttle, AtomicUtil.Direction.COUNTERCLOCKWISE);
    else if (gamepad1.dpad_right)
      rotate(throttle, AtomicUtil.Direction.CLOCKWISE);

    if (Math.abs(gamepad1.left_stick_y) > .05)
      pullUp(scale_motor_power(gamepad1.left_stick_y));

    if (Math.abs(gamepad1.right_stick_y) > .05)
      smoothAim(gamepad1.right_stick_y);

    if (gamepad1.a)
      syncAim();
  }
}

package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

/**
 * Created by davis on 10/10/15.
 */
public class FpsDrive extends AtomicBaseOpMode{
  public void loop() {
    float throttle = gamepad1.left_stick_y;
    float turn = gamepad1.right_stick_x;

    double leftPower = scale_motor_power(throttle - turn);
    double rightPower = scale_motor_power(throttle + turn);

    FL.setPower(leftPower);
    BL.setPower(leftPower);
    FR.setPower(rightPower);
    BR.setPower(rightPower);

    if (gamepad1.a)
      bopper.setPosition(1.0);
    else if (gamepad1.b)
      bopper.setPosition(0.0);
    else
      bopper.setPosition(0.5);

    if (gamepad1.x)
      drop.setPosition(1.0);
    else if (gamepad1.y)
      drop.setPosition(0.0);

    if (gamepad1.left_bumper)
      lift.setPosition(1);
    else if (gamepad1.right_bumper)
      lift.setPosition(0);
    else
      lift.setPosition(.493);

    if (gamepad1.dpad_up) {
      leftZipper.setPosition(0);
      rightZipper.setPosition(1);
    }
    else if (gamepad1.dpad_left)
      leftZipper.setPosition(.5);
    else if (gamepad1.dpad_right)
      rightZipper.setPosition(.5);


    printTelemetry();
  }
}

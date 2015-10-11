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

    if (gamepad1.left_stick_button && throttle > 0)
      belt.setPower(0.75);
    else if (gamepad1.left_stick_button && throttle < 0)
      belt.setPower(-0.75);
    else
      belt.setPower(0.0);

    printTelemetry();
  }
}

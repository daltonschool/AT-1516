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

    if (gamepad1.left_stick_button)
      belt.setPower(scale_motor_power(throttle));
    else
      belt.setPower(0.0);

    if (gamepad1.a)
      bopper.setPosition(1.0);
    else if (gamepad1.b)
      bopper.setPosition(0.0);
    else
      bopper.setPosition(0.5);

    if (gamepad1.x)
      lift.setPosition(1.0);
    else if (gamepad1.y)
      lift.setPosition(0.0);
    else
      lift.setPosition(.493);

    if (gamepad1.left_bumper)
      drop.setPosition(1);
    else if (gamepad1.right_bumper)
      drop.setPosition(0);


    printTelemetry();
  }
}

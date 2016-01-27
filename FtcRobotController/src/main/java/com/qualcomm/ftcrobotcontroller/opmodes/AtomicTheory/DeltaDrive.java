package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by davis on 1/26/16.
 */
public class DeltaDrive extends BaseTeleOp{
  DcMotor left;
  DcMotor right;
  DcMotor pull;
  Servo aim;

  double armPos;

  Servo AFL; // Front Left Arm
  Servo ABL; // Back Left Arm
  Servo AFR; // Front Right Arm
  Servo ABR; // Back Right Arm

  public void init() {
    AFL = hardwareMap.servo.get("AFL");
    ABL = hardwareMap.servo.get("ABL");
    AFR = hardwareMap.servo.get("AFR");
    ABR = hardwareMap.servo.get("ABR");

    armPos = 0;
    moveArms(armPos);
  }

  void moveLeft(double power) {
    left.setPower(power);
  }
  void moveRight(double power) {
    right.setPower(power);
  }

  public void loop() {

    if (gamepad1.right_bumper)
      armPos = scaleServo(armPos + .01);
    else if (gamepad1.left_bumper)
      armPos = scaleServo(armPos - .01);

    moveArms(armPos);
  }

  void moveLeftArm(double pos) {
    AFL.setPosition(pos);
    ABL.setPosition(pos);
  }

  void moveRightArm(double pos) {
    AFR.setPosition(pos);
    ABR.setPosition(pos);
  }

  void moveArms(double pos) {
    moveLeftArm(pos);
    moveRightArm(pos);
  }
}

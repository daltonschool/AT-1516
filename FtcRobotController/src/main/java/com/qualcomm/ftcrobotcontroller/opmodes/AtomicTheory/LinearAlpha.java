package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Hardware;

/**
 * Created by davis on 11/21/15.
 */
public abstract class LinearAlpha extends BaseAuto {
  DcMotor left;
  DcMotor right;
  Servo aim;
  Servo whack;
  double aimCount;
  DcMotor pull;


  Servo leftPersonDropper;
  double leftPersonCount = 0;

  void moveLeft(double power) {
    left.setPower(power);
  }
  void moveRight(double power) {
    right.setPower(power);
  }

  public void setup() {
    left = hardwareMap.dcMotor.get("left");
    right = hardwareMap.dcMotor.get("right");
    pull = hardwareMap.dcMotor.get("pull");
    aim = hardwareMap.servo.get("aim");
    leftPersonDropper = hardwareMap.servo.get("leftPersonServo");
    whack = hardwareMap.servo.get("whack");

    leftPersonDropper.setPosition(leftPersonCount);
    aimCount = .2;
    encoderMotor1 = left;
    encoderMotor2 = right;
    aim.setPosition(aimCount);
    whack.setPosition(.493);
    left.setDirection(DcMotor.Direction.FORWARD);
    right.setDirection(DcMotor.Direction.REVERSE);
  }

  double scaleServo(double d) {
    if (d > 1)
      return 1;
    else if (d < 0)
      return 0;
    else
      return d;
  }
}

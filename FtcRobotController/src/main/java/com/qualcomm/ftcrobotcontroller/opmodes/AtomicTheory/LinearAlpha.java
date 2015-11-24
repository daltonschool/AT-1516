package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by davis on 11/21/15.
 */
public abstract class LinearAlpha extends BaseAuto {
  DcMotor left;
  DcMotor right;
  Servo aim;
  double aimCount;
  DcMotor pull;

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

    aimCount = 0;

    aim.setPosition(aimCount);

    left.setDirection(DcMotor.Direction.FORWARD);
    right.setDirection(DcMotor.Direction.REVERSE);
  }
}

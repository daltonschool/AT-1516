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
    left = encoderMotor1 = hardwareMap.dcMotor.get("left");
    right= encoderMotor2 = hardwareMap.dcMotor.get("right");
    pull = hardwareMap.dcMotor.get("pull");
    aim = hardwareMap.servo.get("aim");
    leftPersonDropper = hardwareMap.servo.get("leftPersonServo");
    whack = hardwareMap.servo.get("whack");
    colorSensor1 = hardwareMap.colorSensor.get("colorSensor1");
    colorSensor2 = hardwareMap.colorSensor.get("colorSensor2");

    leftPersonDropper.setPosition(leftPersonCount);
    aimCount = .2;
    aim.setPosition(aimCount);
    whack.setPosition(.493);
    left.setDirection(DcMotor.Direction.FORWARD);
    right.setDirection(DcMotor.Direction.REVERSE);
  }
}

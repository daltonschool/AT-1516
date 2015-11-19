package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by davis on 11/5/15.
 */
public abstract class AlphaDrive extends BaseTeleOp{

  DcMotor left;
  DcMotor right;
  Servo aim;
  double aimCount;
  DcMotor pull;

  final double AIM_PRESET = .5; // TODO: FIND AIM PRESET

  public void init() {
    left = hardwareMap.dcMotor.get("left");
    right = hardwareMap.dcMotor.get("right");
    pull = hardwareMap.dcMotor.get("pull");
    aim = hardwareMap.servo.get("aim");

    aimCount = 0;

    aim.setPosition(aimCount);

    left.setDirection(DcMotor.Direction.FORWARD);
    right.setDirection(DcMotor.Direction.REVERSE);

    encoderMotor1 = left;
    encoderMotor2 = right;
  }


  void moveLeft(double power) {
    left.setPower(power);
  }
  void moveRight(double power) {
    right.setPower(power);
  }

  void pressA() {
    aimCount = scaleServo(aimCount + .01);
    aim.setPosition(aimCount);
  }
  void pressB() {
    aimCount =  scaleServo(aimCount -.01);
    aim.setPosition(aimCount);
  }

  void syncAim() {
    aimCount = scaleServo(AIM_PRESET);
    aim.setPosition(aimCount);
  }

  void smoothAim(double m) {
    aimCount = scaleServo(aimCount + m/100);
    aim.setPosition(aimCount);
  }

  void pullUp(double m) {
    pull.setPower(m);
  }

  void noAB() { }
  void pressX() {
    pull.setPower(.75);
  }
  void pressY() {
    pull.setPower(-.75);
  }
  void noXY() {
    pull.setPower(0);
  }

  void moveLift(double dir) {}

  void engageZipline(AtomicUtil.Direction d) { }
  void releaseZipline() { }
}

package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

import com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory.AtomicUtil;
import com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory.BaseTeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by davis on 11/5/15.
 */
public abstract class AlphaDrive extends BaseTeleOp {

  DcMotor left;
  DcMotor right;
  Servo aim;
  Servo dump;
  Servo rightZip;
  Servo leftZip;
  double aimCount;
  double dumpCount;
  double leftZipCount;
  double rightZipCount;

  ColorSensor colorSensor1;
  ColorSensor colorSensor2;

  DcMotor pull;

  final double AIM_PRESET_INTO_THIN_AIR = 0.36;
  final double AIM_PRESET_MID_ZONE = 0.33;// Math.PI/10;


  public void init() {
    left = hardwareMap.dcMotor.get("left");
    right = hardwareMap.dcMotor.get("right");
    pull = hardwareMap.dcMotor.get("pull");

    aim = hardwareMap.servo.get("aim");
    leftZip = hardwareMap.servo.get("leftZip");
    rightZip = hardwareMap.servo.get("rightZip");
    dump = hardwareMap.servo.get("dump");

    originalLeft = left.getCurrentPosition();
    originalRight = right.getCurrentPosition();
    aimCount = 0;
    dumpCount = 1;
    rightZipCount = .3;
    leftZipCount = .7;

    aim.setPosition(aimCount);
    dump.setPosition(dumpCount);
    leftZip.setPosition(leftZipCount);
    rightZip.setPosition(rightZipCount);

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

  void aimUp() {
    aimCount = scaleServo(aimCount + .5/300.0);
    aim.setPosition(aimCount);
  }
  void aimDown() {
    aimCount =  scaleServo(aimCount -.5/300.0);
    aim.setPosition(aimCount);
  }

  void syncAim(double preset) {
    aimCount = scaleServo(preset);
    aim.setPosition(aimCount);
  }

  void rightZipDown() {
    rightZipCount += .01;

    if (rightZipCount > 1)
      rightZipCount = 1;
    else if (rightZipCount < 0) rightZipCount = 0;

    rightZip.setPosition(rightZipCount);
  }

  void rightZipUp() {
    rightZipCount -= .05;

    if (rightZipCount > 1)
      rightZipCount = 1;
    else if (rightZipCount < 0) rightZipCount = 0;

    rightZip.setPosition(rightZipCount);
  }

  void leftZipDown() {
    leftZipCount -= .01;

    if (leftZipCount > 1)
      leftZipCount = 1;
    else if (leftZipCount < 0) leftZipCount = 0;

    leftZip.setPosition(leftZipCount);
  }

  void leftZipUp() {
    leftZipCount -= .05;

    if (leftZipCount > 1)
      leftZipCount = 1;
    else if (leftZipCount < 0) leftZipCount = 0;

    leftZip.setPosition(leftZipCount);
  }
  void resetRight() {
    rightZipCount = .3;
    rightZip.setPosition(rightZipCount);
  }
  void resetLeft() {
    leftZipCount = .7;
    leftZip.setPosition(leftZipCount);
  }
  void resetZips() {
    resetRight();
    resetLeft();
  }

  void dumpDown() {
    dumpCount = scaleServo(dumpCount - .02);
    dump.setPosition(dumpCount);
  }

  void dumpUp() {
    dumpCount = scaleServo(dumpCount + .02);
    dump.setPosition(dumpCount);
  }
  void smoothAim(double m) {
    aimCount = scaleServo(aimCount + m/300);
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
  void noLBLT() { };

  void moveLift(double dir) {}

  void engageZipline(AtomicUtil.Direction d) { }
  void releaseZipline() { }

  void writeTelemetry() {
    telemetry.addData("Aim position", aimCount);
    telemetry.addData("Left encoder", getEncoders()[0]);
    telemetry.addData("Right encoder", getEncoders()[1]);
    telemetry.addData("Dropper", dumpCount);
  }
  int originalLeft;
  int originalRight;
  int[] getEncoders() {
    return new int[] {left.getCurrentPosition() - originalLeft, right.getCurrentPosition() - originalRight};
  }
}

package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
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
  Servo leftPersonDropper;
  Servo whack;
  double aimCount;
  double leftPersonCount;

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
    whack = hardwareMap.servo.get("whack");
    leftPersonDropper = hardwareMap.servo.get("leftPersonServo");
    colorSensor1 = hardwareMap.colorSensor.get("colorSensor1");
    colorSensor2 = hardwareMap.colorSensor.get("colorSensor2");

    originalLeft = left.getCurrentPosition();
    originalRight = right.getCurrentPosition();
    aimCount = 0.2;
    leftPersonCount = 0;

    aim.setPosition(aimCount);
    leftPersonDropper.setPosition(leftPersonCount);

    whack.setPosition(.493);
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

  void syncAim(double preset) {
    aimCount = scaleServo(preset);
    aim.setPosition(aimCount);
  }

  void pressLT(double d) {
    double t = .493 + d/2;
    if (t > 1) t = 1;
    else if (t < 0) t = 0;
    whack.setPosition(t);
  }

  void pressLB() {
    leftPersonCount = scaleServo(leftPersonCount - .002);
    leftPersonDropper.setPosition(leftPersonCount);
  }

  void pressRT(double d) {
    double t = .493 - d;
    if (t > 1) t = 1;
    else if (t < 0) t = 0;
    whack.setPosition(t);
  }

  void pressRB() {
    leftPersonCount = scaleServo(leftPersonCount + .002);
    leftPersonDropper.setPosition(leftPersonCount);
  }

  void noPressT() {
    whack.setPosition(.493);
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
    telemetry.addData("Dropper", leftPersonCount);
    telemetry.addData("ColorSensor1 Alpha", colorSensor1.alpha());
    telemetry.addData("ColorSensor2 Alpha", colorSensor2.alpha());
  }
  int originalLeft;
  int originalRight;
  int[] getEncoders() {
    return new int[] {left.getCurrentPosition() - originalLeft, right.getCurrentPosition() - originalRight};
  }
}

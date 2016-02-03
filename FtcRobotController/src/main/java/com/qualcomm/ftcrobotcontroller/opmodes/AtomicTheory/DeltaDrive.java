package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by davis on 1/26/16.
 */
public class DeltaDrive extends BaseTeleOp{
  DcMotor left;
  DcMotor right;

  // Attachments
  DcMotor leftArm;
  DcMotor rightArm;
  DcMotor pull;
  Servo aim;

  double armPos;
  double floorPos;

  Servo floor;

  public void init() {
    leftArm = hardwareMap.dcMotor.get("leftArm");
    rightArm = hardwareMap.dcMotor.get("rightArm");
    rightArm.setDirection(DcMotor.Direction.REVERSE);

    left = hardwareMap.dcMotor.get("left");
    right = hardwareMap.dcMotor.get("right");
    right.setDirection(DcMotor.Direction.REVERSE);
//    floor = hardwareMap.servo.get("floor");

    armPos = 0;
    moveArms(armPos);
    setFloor(.5);
  }

  void moveLeft(double power) {
    left.setPower(power);
  }
  void moveRight(double power) {
    right.setPower(power);
  }

  public void loop() {

    float throttle = gamepad1.left_stick_y;
    float turn = gamepad1.right_stick_x;

    double leftPower = scale_motor_power(throttle - turn);
    double rightPower = scale_motor_power(throttle + turn);

    moveLeft(leftPower);
    moveRight(rightPower);


    if (Math.abs(gamepad1.right_trigger) > .1)
      moveArms(gamepad1.right_trigger);
    else if (Math.abs(gamepad1.left_trigger) > .1)
      moveArms(-gamepad1.left_trigger);
    else
      moveArms(0);


    if(gamepad1.x)
      floorPos = scaleServo(floorPos - .01);
    if(gamepad1.b)
      floorPos = scaleServo(floorPos + .01);
    if(gamepad1.y)
      floorPos = scaleServo(floorPos = .5);

    setFloor(floorPos);
  }

  void moveArms(double pow) {
    pow *= .2;

    leftArm.setPower(pow);
    rightArm.setPower(pow);
  }

  void setFloor(double pos){
//    floor.setPosition(pos);
  }
}

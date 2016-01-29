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
  double floorPos;

  Servo AFL; // Front Left Arm
  Servo ABL; // Back Left Arm
  Servo AFR; // Front Right Arm
  Servo ABR; // Back Right Arm
  Servo floor;

  public void init() {
    AFL = hardwareMap.servo.get("AFL");
    ABL = hardwareMap.servo.get("ABL");
    AFR = hardwareMap.servo.get("AFR");
    ABR = hardwareMap.servo.get("ABR");
    floor = hardwareMap.servo.get("floor");

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

    if (gamepad1.right_bumper)
      armPos = scaleServo(armPos + .01);
    else if (gamepad1.left_bumper)
      armPos = scaleServo(armPos - .01);
    if(gamepad1.x)
      floorPos = scaleServo(floorPos - .01);
    if(gamepad1.b)
      floorPos = scaleServo(floorPos + .01);
    if(gamepad1.y)
      floorPos = scaleServo(floorPos = .5);

    moveArms(armPos);
    setFloor(floorPos);
  }

  void moveLeftArm(double pos) {
    AFL.setPosition(1.0-pos);
    ABL.setPosition(1.0 - pos);
  }

  void moveRightArm(double pos) {
    AFR.setPosition(pos);
    ABR.setPosition(pos);
  }

  void moveArms(double pos) {
    moveLeftArm(pos);
    moveRightArm(pos);
  }

  void setFloor(double pos){
    floor.setPosition(pos);
  }
}

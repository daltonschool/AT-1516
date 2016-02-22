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
  double aimPos;
  Servo floor;
  Servo conveyor;

  DcMotor elevator;
  DcMotor nom;



  public void init() {
    // lift
    leftArm = hardwareMap.dcMotor.get("lift left");
    rightArm = hardwareMap.dcMotor.get("lift right");
    rightArm.setDirection(DcMotor.Direction.REVERSE);

    // drivetrain
    left = hardwareMap.dcMotor.get("drive left");
    right = hardwareMap.dcMotor.get("drive right");
    left.setDirection(DcMotor.Direction.REVERSE);

    // pickup
    elevator = hardwareMap.dcMotor.get("elevator");
    nom = hardwareMap.dcMotor.get("nom");
    floor = hardwareMap.servo.get("box tipper");

    // hang
    pull = hardwareMap.dcMotor.get("pullup");
    pull.setDirection(DcMotor.Direction.REVERSE);
    aim = hardwareMap.servo.get("pullup aim");

    // conveyor test
    conveyor = hardwareMap.servo.get("conveyor");
    conveyor.setPosition(.493);


    floorPos = .5;
    aimPos = 1;
    setFloor(floorPos);
    aim.setPosition(aimPos);
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


    // move lift
    if (Math.abs(gamepad1.right_trigger) > .1)
      moveArms(gamepad1.right_trigger);
    else if (Math.abs(gamepad1.left_trigger) > .1)
      moveArms(-gamepad1.left_trigger);
    else
      moveArms(0);

    // move floor of box
    if(gamepad1.x)
      floorPos = scaleServo(floorPos - .01);
    if(gamepad1.b)
      floorPos = scaleServo(floorPos + .01);
    if(gamepad1.y)
      floorPos = scaleServo(floorPos = .5);
    setFloor(floorPos);

    // move elevator and nom
    if (gamepad1.left_bumper) {
      elevator.setPower(.5);
      nom.setPower(1);
    }
    else if (gamepad1.right_bumper) {
      elevator.setPower(-.5);
      nom.setPower(-1);
    }
    else {
      elevator.setPower(0);
      nom.setPower(0);
    }

    // pullup
    if (gamepad1.dpad_up)
      pull.setPower(1);
    else if (gamepad1.dpad_down)
      pull.setPower(-1);
    else
      pull.setPower(0);

    // servo aim
    if (gamepad1.dpad_left)
      aimUp();
    else if (gamepad1.dpad_right)
      aimDown();

    //move conveyor
    if (gamepad2.dpad_right)
      conveyor.setPosition(1);
    else if (gamepad2.dpad_left)
      conveyor.setPosition(0);
    else
      conveyor.setPosition(.493);
  }

  void moveArms(double pow) {
    pow *= .75;

    leftArm.setPower(pow);
    rightArm.setPower(pow);
  }

  void aimUp() {
    aimPos = scaleServo(aimPos + .5/100.0);
    aim.setPosition(aimPos);
  }
  void aimDown() {
    aimPos =  scaleServo(aimPos -.5/100.0);
    aim.setPosition(aimPos);
  }

  void syncAim(double preset) {
    aimPos = scaleServo(preset);
    aim.setPosition(aimPos);
  }

  void setFloor(double pos){
    floor.setPosition(pos);
  }

}

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

  Servo climberDumper;
  double climberPos;

  Servo ziplinerLeft;
  Servo ziplinerRight;

  double zipLeftPos = .95;
  double zipRightPos = .05;


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
    right.setDirection(DcMotor.Direction.REVERSE);

    // pickup
    elevator = hardwareMap.dcMotor.get("elevator");
    nom = hardwareMap.dcMotor.get("nom");

    // hang
    pull = hardwareMap.dcMotor.get("pullup");
    pull.setDirection(DcMotor.Direction.REVERSE);
    aim = hardwareMap.servo.get("pullup aim");

    // conveyor
    conveyor = hardwareMap.servo.get("conveyor");
    conveyor.setPosition(.493);

    //climber dumper
    climberDumper = hardwareMap.servo.get("climber");

    ziplinerLeft = hardwareMap.servo.get("zipl");
    ziplinerRight = hardwareMap.servo.get("zipr");

    ziplinerLeft.setPosition(.5);
    ziplinerRight.setPosition(.5);

    floorPos = .5;
    aimPos = 0;
    climberPos = 1;

    climberDumper.setPosition(climberPos);
    aim.setPosition(aimPos);
  }

  void moveLeft(double power) {
    left.setPower(power);
  }
  void moveRight(double power) {
    right.setPower(power);
  }

  public void loop() {

    /**
     * Ready: Player One
     */
    // Drive
    float throttle = gamepad1.left_stick_y;
    float turn = gamepad1.right_stick_x;

    double leftPower = scale_motor_power(throttle + turn);
    double rightPower = scale_motor_power(throttle - turn);

    moveLeft(leftPower);
    moveRight(rightPower);

    // Zipline Hitter
    if(gamepad1.right_trigger > .5)
      zipRightPos = scaleServo(zipRightPos + .01);
    if(gamepad1.right_bumper)
      zipRightPos = scaleServo(zipRightPos - .01);
    ziplinerRight.setPosition(zipRightPos);

    if(gamepad1.left_trigger > .5)
      zipLeftPos = scaleServo(zipLeftPos - .01);
    if(gamepad1.left_bumper)
      zipLeftPos = scaleServo(zipLeftPos + .01);
    ziplinerLeft.setPosition(zipLeftPos);


    /**
     * Ready: Player Two
     */
    // move lift
    if (Math.abs(gamepad2.right_trigger) > .1)
      moveArms(gamepad2.right_trigger);
    else if (Math.abs(gamepad2.left_trigger) > .1)
      moveArms(-gamepad2.left_trigger);
    else
      moveArms(0);

    // Climber dumper
    if(gamepad2.dpad_right)
      climberPos = scaleServo(climberPos - .01);
    if(gamepad2.dpad_left)
      climberPos = scaleServo(climberPos + .01);
    if(gamepad2.dpad_up)
      climberPos = scaleServo(climberPos = .5);
    setClimber(climberPos);

    // move elevator and nom
    if (gamepad2.left_bumper) {
      elevator.setPower(.5);
      nom.setPower(1);
    }
    else if (gamepad2.right_bumper) {
      elevator.setPower(-.5);
      nom.setPower(-1);
    }
    else {
      elevator.setPower(0);
      nom.setPower(0);
    }

    // pullup
    if (Math.abs(gamepad2.left_stick_y) > .05)
      pull.setPower(gamepad2.left_stick_y);
    else
      pull.setPower(0);

    // servo aim
    if (gamepad2.right_stick_y > .05)
      aimUp();
    else if (gamepad2.right_stick_y < -.05)
      aimDown();

    //move conveyor
    if (gamepad2.a)
      conveyor.setPosition(1);
    else if (gamepad2.b)
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


  void setClimber(double pos) {
    climberDumper.setPosition(pos);
  }

}

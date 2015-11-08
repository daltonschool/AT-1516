package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by davis on 11/7/15.
 */
public abstract class BaseAuto extends LinearOpMode{
  DcMotor encoderMotor1;
  DcMotor encoderMotor2;

  /**
   * Drive the left side of the robot.
   *
   * @param power from -1.0 to 1.0
   */
  abstract void moveLeft(double power);

  /**
   * Drive the right side of the robot.
   *
   * @param power from -1.0 to 1.0
   */
  abstract void moveRight(double power);

  /**
   * Stop the motors.
   */
  void stopMotors() {
    moveLeft(0);
    moveRight(0);
  }

  /**
   * Drive forward, moving both sides of the robot.
   *
   * @param power from -1.0 to 1.0
   */
  void drive(double power) {
    moveLeft(power);
    moveRight(power);
  }

  /**
   * Rotate the robot.
   *
   * @param power double from 0.0.to 1.0
   * @param dir Direction CLOCKWISE or COUNTERCLOCKWISE
   */
  void rotate(double power, AtomicUtil.Direction dir) {
    int d;
    switch (dir) {
      case CLOCKWISE:
        d = 1;
      case COUNTERCLOCKWISE:
        d = -1;
      default:
        d = 0;
    }
    moveLeft(power*d);
    moveRight(power*d);
  }

  void driveTicks(double power, int ticks) {
    int start = encoderMotor1.getCurrentPosition();
    while (Math.abs(encoderMotor1.getCurrentPosition() - start) < ticks)
      drive(power);
    stopMotors();
  }

  void rotateTicks(double power, AtomicUtil.Direction dir, int ticks) {
    int start = encoderMotor1.getCurrentPosition();
    while(Math.abs(encoderMotor1.getCurrentPosition() - start) < ticks)
      rotate(power, dir);
    stopMotors();
  }

  abstract AtomicUtil.Alliance getTeam();
}
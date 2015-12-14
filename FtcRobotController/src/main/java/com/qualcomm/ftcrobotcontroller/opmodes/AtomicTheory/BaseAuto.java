package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by davis on 11/7/15.
 */
public abstract class BaseAuto extends LinearOpMode{
  DcMotor encoderMotor1;
  DcMotor encoderMotor2;

  ColorSensor colorSensor;

  final int LINE_THRESHOLD = 20;

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
        break;
      case COUNTERCLOCKWISE:
        d = -1;
        break;
      default:
        d = 0;
        break;
    }
    moveLeft(-power * d);
    moveRight(power * d);
  }

  void rotateTicks(double power, AtomicUtil.Direction dir, int ticks) {
    int start = encoderMotor1.getCurrentPosition();
    while(Math.abs(encoderMotor1.getCurrentPosition() - start) < ticks)
      rotate(power, dir);
    stopMotors();
  }

  void dumbTicks(double power, int ticks) {
    int start = encoderMotor1.getCurrentPosition();
    int start2 = encoderMotor2.getCurrentPosition();
    while (Math.abs(encoderMotor1.getCurrentPosition() - start) < ticks
            || Math.abs(encoderMotor2.getCurrentPosition() - start2) < ticks) {
      drive(power);
    }
  }

  void driveTicks(double power, int ticks) {
    int startLeft;
    int posLeft;

    int startRight;
    int posRight;

    startLeft = posLeft = Math.abs(encoderMotor1.getCurrentPosition());
    startRight = posRight = Math.abs(encoderMotor2.getCurrentPosition());

    double pl = power;
    double pr = power;

    double K = .001;
    double C = .02;
    do {
      int diff = posLeft - posRight;

      pl += diff *  sign(diff) * K;
      pr += diff * -sign(diff) * K;
      pl = scale(pl);
      pr = scale(pr);

      moveLeft(pl);
      moveRight(pr);

      posLeft = Math.abs(encoderMotor1.getCurrentPosition());
      posRight = Math.abs(encoderMotor2.getCurrentPosition());
    } while (posLeft - startLeft < ticks || posRight - startRight < ticks);

    stopMotors();
  }

  /**
   * Ensure a number is within bounds for motor controllers.
   * @param d number
   * @return number, as long as it's within -1 and 1. Returns the bound otherwise.
   */
  double scale(double d) {
    if (d > 1) return 1;
    if (d < -1) return -1;
    else return d;
  }

  /**
   * Determine if a number is positive or negative
   * @param d number
   * @return 1 if number is positive, -1 if it's negative
   */
  double sign(double d) {
    if (d >= 0) return 1.0;
    else return -1.0;
  }

  /**
   * Follow a white line, using a color sensor.
   * This is an event-driven method. To continue to follow the line,
   * keep calling this function. When you want to stop, call
   * stopMotors() instead of this function.
   *
   * @param speed power to send to the motors.
   */
  void followLine(float speed) {
    if (colorSensor.alpha() > LINE_THRESHOLD) {
      moveLeft(speed);
    } else {
      moveRight(speed);
    }
  }

  /**
   * Follow a line for a given amount of time.
   * @param millis time to follow the line for, in milliseconds.
   */
  void followLine(float speed, double millis) {
    long t = System.currentTimeMillis();
    while (System.currentTimeMillis() - t < millis)
      followLine(speed);
    stopMotors();
  }

  abstract AtomicUtil.Alliance getTeam();
}

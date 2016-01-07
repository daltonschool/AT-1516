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

  ColorSensor colorSensor1;
  ColorSensor colorSensor2;

  final int LINE_THRESHOLD = 20;

  abstract AtomicUtil.Alliance getTeam();

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

  /**
   * Drive forward until encoder tick threshold is met.
   * @param power
   * @param ticks Number of encoder ticks to travel
   */
  void dumbTicks(double power, int ticks) {
    int start = encoderMotor1.getCurrentPosition();
    int start2 = encoderMotor2.getCurrentPosition();
    while (Math.abs(encoderMotor1.getCurrentPosition() - start) < ticks
            || Math.abs(encoderMotor2.getCurrentPosition() - start2) < ticks) {
      drive(power);
    }
  }

  /**
   * Use a proportional control algorithm to drive a certain number of encoder
   * ticks forward.
   * @param power Base motor power. Motors will run around this speed.
   * @param ticks Number of encoder ticks to travel.
   */
  void driveTicks(double power, int ticks) {
    // Starting positions
    int startLeft;
    int startRight;
    // Current positions
    int posLeft;
    int posRight;
    // Motor power in the proportional control algorithm.
    double pl;
    double pr;

    double K = .001; // proportionality constant

    startLeft = Math.abs(encoderMotor1.getCurrentPosition());
    startRight = Math.abs(encoderMotor2.getCurrentPosition());
    pl = pr = power;

    do {
      // update encoder positions
      posLeft = Math.abs(encoderMotor1.getCurrentPosition());
      posRight = Math.abs(encoderMotor2.getCurrentPosition());
      posLeft = Math.abs(posLeft - startLeft);
      posRight = Math.abs(posRight - startRight);
      telemetry.addData("left", posLeft);
      telemetry.addData("right", posRight);

      int error = posLeft - posRight; // Discrepancy between the two encoders
      double correction = error * K; // Multiply that error by the constant to get the correction.

      // correct the motor speeds
      pl += correction;
      pr -= correction;

      // ensure we're still within the thresholds so we don't throw an error
      pl = scale(pl);
      pr = scale(pr);

      // drive the motors
      moveLeft(pl);
      moveRight(pr);
    } while (posLeft - startLeft < ticks || posRight - startRight < ticks);
    // ^^ do all that while we still have encoder ticks left to run

    stopMotors(); // stop motors before exiting
  }

  /**
   * Ensure a number is within bounds for motor controllers.
   * @param d number
   * @return number, as long as it's within -1 and 1. Returns the bound otherwise.
   */
  double scale(double d) {
    if (d > 1) return 1.0;
    if (d < -1) return -1.0;
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

  boolean seesLine(ColorSensor c) {
    return c.alpha() > LINE_THRESHOLD;
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
    // If two color sensors are connected, use advanced line following.
    if (colorSensor2 != null) {
      if (seesLine(colorSensor1) && seesLine(colorSensor2)) // if both see the line
        drive(speed); // drive straight.
      else if (seesLine(colorSensor1)) // if only the left sensor sees the line
        moveRight(speed); // turn to the right
      else if (seesLine(colorSensor2)) // if only the right sensor sees the line
        moveLeft(speed); // turn to the left
      else // if neither sensor sees the line,
        drive(speed); // drive straight.
    }

    // If only one sensor is connected, fallback to simple line following.
    else {
      if (seesLine(colorSensor1)) // if the sensor is on the line
        moveLeft(speed); // move off the line
      else // if the sensor is off the line
        moveRight(speed); // move onto it
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
}

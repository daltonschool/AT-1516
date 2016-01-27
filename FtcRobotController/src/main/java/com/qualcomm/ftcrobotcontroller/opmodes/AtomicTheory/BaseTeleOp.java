package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory.AtomicUtil.*;

/**
 * Created by davis on 11/7/15.
 */
public abstract class BaseTeleOp extends OpMode{
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
  void rotate(double power, Direction dir) {
    int d;
    switch (dir) {
      case CLOCKWISE:
        d = 1;
      case COUNTERCLOCKWISE:
        d = -1;
      default:
        d = 0;
    }
    moveLeft(-power*d);
    moveRight(power*d);
  }


  double scaleServo(double d) {
    if (d > 1)
      return 1;
    else if (d < 0)
      return 0;
    else
      return d;
  }

  static double scale_motor_power (double p_power) {
    //
    // Assume no scaling.
    //
    double l_scale = 0.0f;

    //
    // Ensure the values are legal.
    //
    double l_power = Range.clip(p_power, -1, 1);

    double[] l_array =
            { 0.00, 0.05, 0.09, 0.10, 0.12
                    , 0.15, 0.18, 0.24, 0.30, 0.36
                    , 0.43, 0.50, 0.60, 0.72, 0.85
                    , 1.00, 1.00
            };

    //
    // Get the corresponding index for the specified argument/parameter.
    //
    int l_index = (int) (l_power * 16.0);
    if (l_index < 0)
    {
      l_index = -l_index;
    }
    else if (l_index > 16)
    {
      l_index = 16;
    }

    if (l_power < 0)
    {
      l_scale = -l_array[l_index];
    }
    else
    {
      l_scale = l_array[l_index];
    }

    return l_scale;

  }
}

package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by davis on 11/5/15.
 */
public class AlphaDrive extends OpMode{

  DcMotor left;
  DcMotor right;

  public void init() {
    left = hardwareMap.dcMotor.get("left");
    right = hardwareMap.dcMotor.get("right");

    left.setDirection(DcMotor.Direction.FORWARD);
    right.setDirection(DcMotor.Direction.REVERSE);
  }

  public void loop() {
    float leftSide = gamepad1.left_stick_y;
    float rightSide = gamepad1.right_stick_y;

    left.setPower(scale_motor_power(leftSide));
    right.setPower(scale_motor_power(rightSide));
  }

  static double scale_motor_power (double p_power)
  {
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

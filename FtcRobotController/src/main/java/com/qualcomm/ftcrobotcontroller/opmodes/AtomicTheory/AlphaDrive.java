package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by davis on 11/5/15.
 */
public abstract class AlphaDrive extends BaseTeleOp{

  DcMotor left;
  DcMotor right;

  public void init() {
    left = hardwareMap.dcMotor.get("left");
    right = hardwareMap.dcMotor.get("right");

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

  void pressA() { }
  void pressB() { }
  void noAB() { }
  void pressX() { }
  void pressY() { }
  void noXY() { }
  void moveLift(double dir) {}

  void engageZipline(AtomicUtil.Direction d) { }
  void releaseZipline() { }
}

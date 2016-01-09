package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Hardware;

/**
 * Created by davis on 11/21/15.
 */
public abstract class LinearAlpha extends BaseAuto {
  DcMotor left;
  DcMotor right;
  Servo aim;
  Servo dump;
  Servo rightZip;
  Servo leftZip;
  double aimCount;
  double dumpCount;
  double leftZipCount;
  double rightZipCount;
  DcMotor pull;




  void moveLeft(double power) {
    left.setPower(power);
  }
  void moveRight(double power) {
    right.setPower(power);
  }

  public void setup() {
    left = encoderMotor1 = hardwareMap.dcMotor.get("left");
    right= encoderMotor2 = hardwareMap.dcMotor.get("right");
    pull = hardwareMap.dcMotor.get("pull");
    aim = hardwareMap.servo.get("aim");
    leftZip = hardwareMap.servo.get("leftZip");
    rightZip = hardwareMap.servo.get("rightZip");
    dump = hardwareMap.servo.get("dump");

    aimCount = 0;
    dumpCount = 1;
    rightZipCount = .3;
    leftZipCount = .7;

    aim.setPosition(aimCount);
    dump.setPosition(dumpCount);
    leftZip.setPosition(leftZipCount);
    rightZip.setPosition(rightZipCount);

    left.setDirection(DcMotor.Direction.FORWARD);
    right.setDirection(DcMotor.Direction.REVERSE);

    long systemTime = System.nanoTime();
    try {
      boschBNO055 = new AdafruitIMU(hardwareMap, "bno055"
              , (byte)(AdafruitIMU.BNO055_ADDRESS_A * 2)
              , (byte)AdafruitIMU.OPERATION_MODE_IMU);
    } catch (RobotCoreException e){
      telemetry.addData("FtcRobotController", "Exception: " + e.getMessage());
    }
    telemetry.addData("FtcRobotController", "IMU Init method finished in: "
            + (-(systemTime - (systemTime = System.nanoTime()))) + " ns.");
  }
}

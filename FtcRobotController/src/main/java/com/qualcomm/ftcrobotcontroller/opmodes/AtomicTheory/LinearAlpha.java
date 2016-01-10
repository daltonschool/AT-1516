package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

import android.util.Log;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Hardware;

/**
 * Created by davis on 11/21/15.
 */
public abstract class LinearAlpha extends BaseAuto {

  double turnPower = .65;
  double drivePower = 65;

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
    right = encoderMotor2 = hardwareMap.dcMotor.get("right");
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



    /* gyro initialization */
    curHeading = 0; //CHANGE BASED ON PROGRAM
    hasStarted = false;
    prevHeading = curHeading;

    systemTime = System.nanoTime();
    prevTime = systemTime;
    try {
      gyro = new AdafruitIMU(hardwareMap, "bno055"
              , (byte) (AdafruitIMU.BNO055_ADDRESS_A * 2)//By convention the FTC SDK always does 8-bit I2C bus
              , (byte) AdafruitIMU.OPERATION_MODE_IMU);
    } catch (RobotCoreException e) {
      Log.i("FtcRobotController", "Exception: " + e.getMessage());
    }

    systemTime = System.nanoTime();
    gyro.startIMU();//Set up the IMU as needed for a continual stream of I2C reads.
    Log.i("FtcRobotController", "IMU Start method finished in: "
            + (-(systemTime - (systemTime = System.nanoTime()))) + " ns.");
    updateHeading(); // get the ball rolling with updating gyro?
    //hopefully reduces lag during initial autonomous
  }

  /*gyro specific stuff*/

  AdafruitIMU gyro;

  //The following arrays contain both the Euler angles reported by the IMU (indices = 0) AND the
  // Tait-Bryan angles calculated from the 4 components of the quaternion vector (indices = 1)
  volatile double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2], accs = new double[3];

  boolean hasStarted;

  long systemTime;//Relevant values of System.nanoTime
  long elapsedTime;
  long prevTime;

  /* heading is from -180 to 180 degrees */

  double curHeading;
  double prevHeading;
  double desiredHeading;

  public void turnToHeading(double desiredHeading) {
    updateHeading();
    if (curHeading > desiredHeading) {
      /* might need a dead zone for turning... */
      //Turn left until robot reaches the desiredHeading
      while (curHeading > desiredHeading) {
        updateHeading();
        moveLeft(turnPower);
        moveRight(-turnPower);
      }
      stopMotors();
    } else {
      //Turn right until robot reaches the desiredHeading
      while (curHeading < desiredHeading) {
        updateHeading();
        moveLeft(turnPower);
        moveRight(-turnPower);
      }
      stopMotors();
    }
    updateHeading();
  }

  /**
   * Drive forward until encoder tick threshold is met.
   * @param power
   * @param ticks Number of encoder ticks to travel
   */
  public void driveTicksStraight(double power, int ticks) {
    int start = encoderMotor1.getCurrentPosition();
    int start2 = encoderMotor2.getCurrentPosition();

    double initHeading = curHeading;
    double error_const = .04;


    while (Math.abs(encoderMotor1.getCurrentPosition() - start) < ticks
            || Math.abs(encoderMotor2.getCurrentPosition() - start2) < ticks) {
      updateHeading();

      //gyro is too finicky to do integral stuff so just the basic derivative stuff
      double pl = power;
      double pr = power;


      double error = curHeading - initHeading;

      pl-=error * error_const;
      pr+=error * error_const;

      pl = scale(pl);
      pr = scale(pr);

      moveLeft(pl);
      moveRight(pr);
      telemetry.addData("m1:", Math.abs(encoderMotor1.getCurrentPosition() - start) - ticks);
      telemetry.addData("m2:", Math.abs(encoderMotor2.getCurrentPosition() - start2) - ticks);
    }

    stopMotors();
  }

  public void updateHeading() {
    elapsedTime = systemTime - prevTime;
    prevTime = systemTime;
    double elapsedSeconds = elapsedTime / 1000000000;
    systemTime = System.nanoTime();

    //Update gyro values
    gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
    prevHeading = curHeading;
    curHeading = yawAngle[0];

    //Display information on screen
    telemetry.addData("Headings(yaw): ",
            String.format("Euler= %4.5f", yawAngle[0]));

  }
}
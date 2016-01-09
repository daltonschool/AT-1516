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

  double turnPower = .5;
  double drivePower = .5;

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



    /* gyro initialization */
    curHeading = 0; //CHANGE BASED ON PROGRAM
    hasStarted = false;
    prevHeading = curHeading;
    curXAcc = 0;
    curYAcc = 0;
    curZAcc = 0;
    prevXAcc = 0;
    prevYAcc = 0;
    prevZAcc = 0;

    curXVel = 0;
    curYVel = 0;
    curZVel = 0;
    prevXVel = 0;
    prevYVel = 0;
    prevZVel = 0;

    curXPos = 0; //CHANGE BASED ON PROGRAM
    curYPos = 0; //CHANGE BASED ON PROGRAM
    curZPos = 0;
    prevXPos = curXPos;
    prevYPos = 0;
    prevZVel = 0;


    systemTime = System.nanoTime();
    prevTime = systemTime;
    try
    {
      gyro = new AdafruitIMU(hardwareMap, "bno055"
              , (byte)(AdafruitIMU.BNO055_ADDRESS_A * 2)//By convention the FTC SDK always does 8-bit I2C bus
              , (byte)AdafruitIMU.OPERATION_MODE_IMU);
    } catch (RobotCoreException e){
      Log.i("FtcRobotController", "Exception: " + e.getMessage());
    }
  }

  /*gyro specific stuff*/

  AdafruitIMU gyro;

  //The following arrays contain both the Euler angles reported by the IMU (indices = 0) AND the
  // Tait-Bryan angles calculated from the 4 components of the quaternion vector (indices = 1)
  volatile double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2], accs = new double[3];
  double curXAcc; //In m/s^2
  double curYAcc; //In m/s^2
  double curZAcc; //In m/s^2
  double prevXAcc; //In m/s^2
  double prevYAcc; //In m/s^2
  double prevZAcc; //In m/s^2

  double curXVel; //In m/s
  double curYVel; //In m/s
  double curZVel; //In m/s
  double prevXVel; //In m/s
  double prevYVel; //In m/s
  double prevZVel; //In m/s

  double curXPos; //In cm
  double curYPos; //In cm
  double curZPos; //In cm
  double prevXPos; //In cm
  double prevYPos; //In cm
  double prevZPos; //In cm

  boolean hasStarted;

  long systemTime;//Relevant values of System.nanoTime
  long elapsedTime;
  long prevTime;

  /* heading is from -180 to 180 degrees */

  double curHeading;
  double prevHeading;
  double desiredHeading;

  public void turnToHeading(double desiredHeading)
  {
    updatePosition();
    if(curHeading > desiredHeading)
    {
      /* might need a dead zone for turning... */
      //Turn left until robot reaches the desiredHeading
      while(curHeading > desiredHeading)
      {
        updatePosition();
        moveLeft(turnPower);
        moveRight(-turnPower);
      }
      stopMotors();
    }
    else
    {
      //Turn right until robot reaches the desiredHeading
      while(curHeading < desiredHeading)
      {
        updatePosition();
        moveLeft(turnPower);
        moveRight(-turnPower);
      }
      stopMotors();
    }
    updatePosition();
  }

  public void updatePosition()
  {
    elapsedTime = systemTime - prevTime;
    prevTime = systemTime;
    double elapsedSeconds = elapsedTime / 1000000000;
    systemTime = System.nanoTime();

    //Update accelerations
    gyro.getAccel(accs);
    prevXAcc = curXAcc;
    prevYAcc = curYAcc;
    prevZAcc = curZAcc;
    curXAcc = accs[0];
    curYAcc = accs[1];
    curZAcc = accs[2];

    //Update velocities
    prevXVel = curXVel;
    prevYVel = curYVel;
    prevZVel = curZVel;
    curXVel = prevXVel + (curXAcc / elapsedSeconds);
    curYVel = prevYVel + (curYAcc / elapsedSeconds);
    curZVel = prevZVel + (curZAcc / elapsedSeconds);

    //Update position
    prevXPos = curXPos;
    prevYPos = curYPos;
    prevZPos = curZPos;
    curXPos = prevXPos + ((curXVel / elapsedSeconds) * 100);
    curYPos = prevYPos + ((curYVel / elapsedSeconds) * 100);
    curZPos = prevZPos + ((curZPos / elapsedSeconds) * 100);

    //Update gyro values
    gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
    prevHeading = curHeading;
    curHeading = yawAngle[0];

    //Display information on screen
    telemetry.addData("Headings(yaw): ",
            String.format("Euler= %4.5f", yawAngle[0]));
  }
}

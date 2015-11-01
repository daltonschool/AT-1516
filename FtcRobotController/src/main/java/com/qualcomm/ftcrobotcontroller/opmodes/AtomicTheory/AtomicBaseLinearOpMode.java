package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import static com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory.AtomicUtil.*;
import com.qualcomm.ftcrobotcontroller.opmodes.cheer4ftc.LinearOpModeCamera;
/**
 * Created by Davis Haupt on 10/2/15.
 */
public abstract class AtomicBaseLinearOpMode extends LinearOpMode {

  DeviceInterfaceModule cdim;

  //Motor Controllers
  DcMotorController front_motor_controller_drive;
  DcMotorController back_motor_controller_drive;

  //Servo Controller(s)
  ServoController servoController1;

  //Drive Motors
  DcMotor BR;
  DcMotor BL;
  DcMotor FR;
  DcMotor FL;

  // Servos
  Servo bopper;
  Servo lift;
  Servo drop;

  //Sensors
  ColorSensor colorSensor1;
  ColorSensor colorSensor2;



  public void config() {
    //Configure Motor Controllers
    back_motor_controller_drive = hardwareMap.dcMotorController.get("Motor Controller 1");
    front_motor_controller_drive = hardwareMap.dcMotorController.get("Motor Controller 2");

    cdim = hardwareMap.deviceInterfaceModule.get("dim");

    //Configure Servo Controller(s)
    servoController1 = hardwareMap.servoController.get("Servo Controller 1");

    //Configure Sensors
    colorSensor1 = hardwareMap.colorSensor.get("colorSensor1");
    colorSensor2 = hardwareMap.colorSensor.get("colorSensor2");

    // Configure Servos
    bopper = hardwareMap.servo.get("bopper");
    lift = hardwareMap.servo.get("lift");
    drop = hardwareMap.servo.get("drop");

    //Set Servos
    lift.setPosition(.493);lift.setDirection(Servo.Direction.FORWARD);
    drop.setPosition(1.0);
    bopper.setPosition(0.5);

    //Configure Motors
    BR = hardwareMap.dcMotor.get("back_right");
    BL = hardwareMap.dcMotor.get("back_left");
    FR = hardwareMap.dcMotor.get("front_right");
    FL = hardwareMap.dcMotor.get("front_left");

    //Set Left Motors as FORWARD
    BL.setDirection(DcMotor.Direction.REVERSE);
    FL.setDirection(DcMotor.Direction.REVERSE);

    //Set Right Motors as REVERSE
    BR.setDirection(DcMotor.Direction.FORWARD);
    FR.setDirection(DcMotor.Direction.FORWARD);

  }

  /**
   *
   * @return the team that the player is on. (RED or BLUE)
   */
  //public abstract Alliance getTeam();

  /**
   * Stop all motors.
   */
  public void stopMotors() {
    BR.setPower(0);
    BL.setPower(0);
    FR.setPower(0);
    FL.setPower(0);
  }

  /**
   * Drive the robot forwards or backwards at a certain power.
   * @param pow
   * @param dir
   */
  public void drive(double pow, Direction dir) {
    int d;
    switch(dir) {
      case FORWARD:
        d = 1;
        break;
      case BACKWARD:
        d = -1;
        break;
      default:
        d = 0;
    }

    FL.setPower(pow*d);
    FR.setPower(pow*d);
    BL.setPower(pow*d);
    BR.setPower(pow*d);
  }


  /**
   * Drive the robot for a certain number of encoder ticks.
   * @param ticks number of encoder ticks
   * @param pow motor power (0.0-1.0)
   * @param dir direction of motion
   */
  public void driveTicks(int ticks, double pow, Direction dir) {
    int startTicks = BR.getCurrentPosition();
    while (Math.abs(BR.getCurrentPosition() - startTicks) < ticks)
      drive(pow, dir);

    stopMotors();
  }

  /**
   * Rotate (Left side one direction, right side other) at a certain power.
   * @param pow
   * @param dir
   */
  public void rotate(double pow, Direction dir) {
    int d;
    switch(dir){
      case CLOCKWISE:
        d = 1;
      case COUNTERCLOCKWISE:
        d = -1;
      default:
        d = 0;
    }

    FL.setPower(pow*d*-1);
    BL.setPower(pow*d*-1);
    FR.setPower(pow*d);
    BR.setPower(pow * d);
  }

  /**
   * Rotate the robot for a certain number of encoder ticks.
   * @param ticks
   * @param pow
   * @param dir
   */
  public void rotateTicks(int ticks, double pow, Direction dir) {
    int startTicks = BR.getCurrentPosition();
    while (Math.abs(BR.getCurrentPosition() - startTicks) < ticks)
      rotate(pow, dir);
    stopMotors();
  }
}

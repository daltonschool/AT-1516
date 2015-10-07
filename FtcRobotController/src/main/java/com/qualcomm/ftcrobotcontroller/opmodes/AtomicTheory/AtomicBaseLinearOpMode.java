package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import com.qualcomm.ftcrobotcontroller.opmodes.cheer4ftc.LinearOpModeCamera;
/**
 * Created by Davis Haupt on 10/2/15.
 */
public abstract class AtomicBaseLinearOpMode extends LinearOpModeCamera {

  public enum Alliance {
    RED, BLUE;
  }

  static final int LEFT = -1;
  static final int RIGHT = 1;

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

  /**
   *
   * @return the team that the player is on. (RED or BLUE)
   */
  public abstract Alliance getTeam();

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

    //Configure Servos
    bopper = hardwareMap.servo.get("bopper");


    //Configure Motors
    BR = hardwareMap.dcMotor.get("back_right");
    BL = hardwareMap.dcMotor.get("back_left");
    FR = hardwareMap.dcMotor.get("front_right");
    FL = hardwareMap.dcMotor.get("front_left");

    //Set Left Motors as FORWARD
    BL.setDirection(DcMotor.Direction.FORWARD);
    FL.setDirection(DcMotor.Direction.FORWARD);

    //Set Right Motors as REVERSE
    BR.setDirection(DcMotor.Direction.REVERSE);
    FR.setDirection(DcMotor.Direction.REVERSE);
  }

  /**
   * Drive the robot for a certain number of encoder ticks.
   * @param ticks number of encoder ticks
   * @param pow motor power (0.0-1.0)
   * @param dir direction of motion
   */
  public void driveTicks(int ticks, double pow, DcMotor.Direction dir) {
    int d = (dir == DcMotor.Direction.FORWARD) ? 1 : -1;
  }

  /**
   * Drive the robot forwards or backwards at a certain power.
   * @param pow
   * @param dir
   */
  public void drive(double pow, DcMotor.Direction dir) {
    int d = (dir == DcMotor.Direction.FORWARD) ? 1 : -1;
  }

  /**
   * Stop all motors.
   */
  public void stopMotors() {
    BR.setPower(0);
    BL.setPower(0);
    FR.setPower(0);
    FL.setPower(0);
  }

}

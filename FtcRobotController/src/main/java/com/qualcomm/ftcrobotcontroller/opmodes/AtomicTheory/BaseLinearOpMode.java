package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

/**
 * Created by Davis Haupt on 10/2/15.
 */
public abstract class BaseLinearOpMode extends LinearOpMode{
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

}

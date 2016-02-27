package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
/**
 * Created by Nathaniel Ostrer on 9/26/15.
 *
 * AtomicBaseOpMode.java is the Atomic Theory Helper Class
 */


public abstract class AtomicBaseOpMode extends OpMode {

    //So like the loop method is unique in the Teleop classes
    //and the init? method is unique in the Autonomous classes

    DeviceInterfaceModule cdim;

    //Motor Controllers
    DcMotorController front_motor_controller_drive;
    DcMotorController back_motor_controller_drive;
    DcMotorController nxtmotor;
    //Servo Controller(s)
    ServoController servoController1;

    //Drive Motors
    DcMotor BR;
    DcMotor BL;
    DcMotor FR;
    DcMotor FL;

    DcMotor belt;
    //Servos
    Servo bopper;
    Servo lift;
    Servo drop;
    Servo leftZipper;
    Servo rightZipper;

    //Sensors
    ColorSensor colorSensor1;
    ColorSensor colorSensor2;


    //Initialize hardware
    public void init() {
        // Configure Servos
        bopper = hardwareMap.servo.get("bopper");
        lift = hardwareMap.servo.get("lift");
        drop = hardwareMap.servo.get("drop");
        leftZipper = hardwareMap.servo.get("leftZipper");
        rightZipper = hardwareMap.servo.get("rightZipper");

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

    public void printTelemetry() {
        telemetry.addData("Left motor power", BL.getPower());
        telemetry.addData("Right motor power", BR.getPower());
        telemetry.addData("Encoder distance traveled", BL.getCurrentPosition());
    }

    public void printi2cData(String sensorName, int port) {
        byte[] read = cdim.getCopyOfReadBuffer(port);
        for (int i = 0; i < read.length; i++)
            telemetry.addData(sensorName + " #"+i, read[i]);
    }

    //scales motor power to
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

    public static String toHexString(int argb) {
        String a = Integer.toHexString(argb & 0xff000000);
        String r = Integer.toHexString(argb & 0x00ff0000);
        String g = Integer.toHexString(argb & 0x0000ff00);
        String b = Integer.toHexString(argb & 0x000000ff);

        return a+r+g+b;
    }



}
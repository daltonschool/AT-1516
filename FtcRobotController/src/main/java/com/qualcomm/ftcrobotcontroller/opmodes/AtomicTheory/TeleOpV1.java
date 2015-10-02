package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;

import com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory.*;

/**
 * Created by Nathaniel Ostrer on 9/24/15.
 */

public class TeleOpV1 extends AtomSplitter {

    public void loop() {

        //Get the values from the joypad
        float throttleLeft = -gamepad1.left_stick_y;
        float throttleRight = -gamepad1.right_stick_y;


        double leftPower = (float) scale_motor_power(throttleLeft);
        double rightPower = (float) scale_motor_power(throttleRight);


        // write the values to the motors
        BR.setPower(rightPower);
        FR.setPower(rightPower);

        BL.setPower(leftPower);
        FL.setPower(leftPower);

        if (gamepad1.a) {
            colorSensor1.enableLed(true);
            colorSensor2.enableLed(true);
        }
        else if (gamepad1.b) {
            colorSensor1.enableLed(false);
            colorSensor2.enableLed(false);
        }

		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
        telemetry.addData("Atomic Theory 4174", "*** Robot Data***");
        printi2cData("Color Sensor", 1);
    }


}

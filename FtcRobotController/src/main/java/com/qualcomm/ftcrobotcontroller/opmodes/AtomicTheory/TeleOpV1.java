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


		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
        telemetry.addData("Atomic Theory 4174", "*** Robot Data***");
        telemetry.addData("right_motors", "right_motors:  " + String.format("%.2f", BR.getPower()));
        telemetry.addData("left_motors", "left_motors:  " + String.format("%.2f", BL.getPower()));
        telemetry.addData("Color sensor1: ", "argb: " + toHexString(colorSensor1.argb()));
        telemetry.addData("Color sensor2: ", "argb: " + toHexString(colorSensor2.argb()));
    }


}

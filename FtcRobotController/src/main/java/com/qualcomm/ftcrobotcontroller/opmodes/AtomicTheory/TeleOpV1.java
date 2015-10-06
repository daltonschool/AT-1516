package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

/**
 * Created by Nathaniel Ostrer on 9/24/15.
 */

public class TeleOpV1 extends AtomicBaseOpMode {

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
            bopper.setPosition(0.5);
        }
        else if (gamepad1.x) {
            bopper.setPosition(0.0);
        }
        else if (gamepad1.y) {
            bopper.setPosition(1.0);
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

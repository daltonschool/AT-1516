package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

/**
 * Created by Nathaniel Ostrer on 9/24/15.
 */

public class TeleOpV1 extends AtomicBaseOpMode {

    public void loop() {

        //Get the values from the joynj pad
        float throttleLeft = -gamepad1.left_stick_y;
        float throttleRight = -gamepad1.right_stick_y;

        double leftPower = (float) scale_motor_power(throttleLeft);
        double rightPower = (float) scale_motor_power(throttleRight);


        // write the values to the motors
        BR.setPower(rightPower);
        FR.setPower(rightPower);

        BL.setPower(leftPower);
        FL.setPower(leftPower);

        if (gamepad1.left_bumper)
            belt.setPower(.75);
        else if (gamepad1.right_bumper)
            belt.setPower(-.75);
        else
            belt.setPower(0);

        if (gamepad1.a) {

        }
        if (gamepad1.b) {
            bopper.setPosition(0.5);
        }
        if (gamepad1.x) {
            bopper.setPosition(0.0);
        }
        if (gamepad1.y) {
            bopper.setPosition(1.0);
        }

		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
        telemetry.addData("Atomic Theory 4174", "*** Robot Data***");
    }


}

package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by nathaniel on 1/29/16.
 */
public class MoveOneMotor extends OpMode {

    DcMotor motor;

    public void init() {
        motor = hardwareMap.dcMotor.get("motor");
    }

    public void loop() {
        if(Math.abs(gamepad1.left_stick_y) > .2) {
            motor.setPower(gamepad1.left_stick_y);
        } else {
            motor.setPower(0);
        }
    }
}

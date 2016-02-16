package org.usfirst.ftc.exampleteam.yourcodehere;

import com.qualcomm.robotcore.hardware.*;
import org.swerverobotics.library.*;
import org.swerverobotics.library.interfaces.*;

/**
 * A skeletal example of a do-nothing first OpMode. Go ahead and change this code
 * to suit your needs, or create sibling OpModes adjacent to this one in the same
 * Java package.
 */
@TeleOp(name="ServoTest")

public class ServoTest extends SynchronousOpMode
{
    /* Declare here any fields you might find useful. */
    // DcMotor motorLeft = null;
    // DcMotor motorRight = null;

    @Override public void main() throws InterruptedException
    {
        /* Initialize our hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names you assigned during the robot configuration
         * step you did in the FTC Robot Controller app on the phone.
         */
        // this.motorLeft = this.hardwareMap.dcMotor.get("motorLeft");
        // this.motorRight = this.hardwareMap.dcMotor.get("motorRight");

        // Wait for the game to start
        waitForStart();

        Servo l1 = hardwareMap.servo.get("l1");
        Servo l2 = hardwareMap.servo.get("l2");

        Servo r1 = hardwareMap.servo.get("r1");
        Servo r2 = hardwareMap.servo.get("r2");
        r1.setDirection(Servo.Direction.REVERSE);
        r2.setDirection(Servo.Direction.REVERSE);

        double servoPos = 0.1;

        // Go go gadget robot!
        while (opModeIsActive())
        {
            if (updateGamepads())
            {
                // The game pad state has changed. Do something with that!
                double d = gamepad1.left_stick_y;

                if(Math.abs(d) > .3) {
                    servoPos += d*.1;
                }

                if(servoPos > .9)
                    servoPos = .9;
                else if(servoPos < 0.1)
                    servoPos = .1;

                r1.setPosition(servoPos);
                r2.setPosition(servoPos);

                l1.setPosition(servoPos);
                l2.setPosition(servoPos);

            }


            telemetry.addData("servoPos: ", servoPos);


            telemetry.update();
            idle();
        }
    }
}

package org.usfirst.ftc.exampleteam.yourcodehere;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.swerverobotics.library.*;
import org.swerverobotics.library.interfaces.*;


/**
 * SynchIMUDemo gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 * http://www.adafruit.com/products/2472
 */
@TeleOp(name="Autonomous")

public class Autonomous extends SynchronousOpMode
{
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    // Our sensors, motors, and other devices go here, along with other long term state
    IBNO055IMU              imu;
    ElapsedTime             elapsed    = new ElapsedTime();
    IBNO055IMU.Parameters   parameters = new IBNO055IMU.Parameters();

    // Here we have state we use for updating the dashboard. The first of these is important
    // to read only once per update, as its acquisition is expensive. The remainder, though,
    // could probably be read once per item, at only a small loss in display accuracy.
    EulerAngles angles;
    Position position;
    int                     loopCycles;
    int                     i2cCycles;
    double                  ms;

    //----------------------------------------------------------------------------------------------
    // main() loop
    //----------------------------------------------------------------------------------------------

    //
    DcMotor leftMotor;
    DcMotor rightMotor;

    DcMotorController driveMotorController;
    AnalogInput dist;

    double pl = 0;
    double pr = 0;

    double drive_straight_p_error_const = .03; //proportional error_const for driving straight

    double[] distReadings = new double[5]; //the past N (5) distance readings for the distance sensor, we throw out
                                            //outliers when we average
    int numDistReadings = 0;

    @Override public void main() throws InterruptedException
    {
        // We are expecting the IMU to be attached to an I2C port on a core device interface
        // module and named "imu". Retrieve that raw I2cDevice and then wrap it in an object that
        // semantically understands this particular kind of sensor.
        parameters.angleUnit      = IBNO055IMU.ANGLEUNIT.DEGREES;
        parameters.accelUnit      = IBNO055IMU.ACCELUNIT.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.loggingTag     = "BNO055";
        imu = ClassFactory.createAdaFruitBNO055IMU(hardwareMap.i2cDevice.get("bno055"), parameters);


        dist = hardwareMap.analogInput.get("dist");

        // Enable reporting of position using the naive integrator
        imu.startAccelerationIntegration(new Position(), new Velocity());

        // Set up our dashboard computations
        composeDashboard();

        this.leftMotor = hardwareMap.dcMotor.get("left");
        this.rightMotor = hardwareMap.dcMotor.get("right");
        this.driveMotorController = hardwareMap.dcMotorController.get("Motor Controller 1");

        this.leftMotor.setDirection(DcMotor.Direction.REVERSE);



        // Wait until we're told to go
        waitForStart();

//        boolean headingAngleWorked = false;
//
//        while(!headingAngleWorked) {
//            try {
//                double a = angles.heading;
//                headingAngleWorked = true;
//            } catch (Exception e) {}
//        }
//
//


//        Thread t = new Thread() {
//            public void run() {
//                turnDegrees(90, .33);
//            }
//        };
//
//        t.run();

        //turnDegrees(90, .4);

        while(getDistance() == Double.MAX_VALUE);

        driveUntilDistEqualsStraight(.5, 1.47);

        turnDegrees(-90, .4);
        driveUntilDistEqualsStraight(.5, .2);

        while(true) {
            telemetry.addData("fin", "");
            telemetry.update();
        }
    }

    //----------------------------------------------------------------------------------------------
    // dashboard configuration
    //----------------------------------------------------------------------------------------------

    double getDistance() {
        distReadings[numDistReadings%distReadings.length] = dist.getValue() * 5.0 / 512.0; // convert to meters

        numDistReadings++;

        if(numDistReadings < distReadings.length) {
            return Double.MAX_VALUE;
        } else {
            double sum = 0.0;

            for(int i = 0; i < distReadings.length; i++) {
                sum += distReadings[i];
            }

            telemetry.addData("total", sum);
            telemetry.addData("numreadings", numDistReadings);

            return sum / (double) distReadings.length;
        }
    }

    void turnDegrees(double degrees, double motorPower) {
        double current_heading = 0;

        try {
            current_heading = angle360(angles.heading);
        } catch(Exception e) {}

        turnToHeading((current_heading + degrees + 360) % 360, motorPower);
    }

    void turnToHeading(double target_heading, double motorPower) {
        target_heading = angle360(target_heading);

        double current_heading = 0;

        try{
            current_heading = angle360(angles.heading);
        } catch(Exception e) {}


        double turnLeftDegrees = Math.abs(current_heading + 360 - target_heading);
        double turnRightDegrees = Math.abs(target_heading-current_heading);

        double degreeDiffThreshold = 5;


        double initDiff = Math.abs(current_heading - target_heading);


        if(turnLeftDegrees < turnRightDegrees) {
            pl = -motorPower;
            pr = motorPower;
        } else {
            pl = motorPower;
            pr = -motorPower;
        }


        long prevTime = System.nanoTime();


        do {
            telemetry.addData("time diff", (double) (System.nanoTime()-prevTime) / (double) 1000000000);
            prevTime = System.nanoTime();

            try {
                current_heading = angle360(angles.heading);
            } catch(Exception e) {};


            double scalingfactor = Math.abs(current_heading - target_heading)/initDiff;

//            if(scalingfactor < .65)
//                scalingfactor = .65;

//            if(motorPower * scalingfactor < .3)
//                scalingfactor = .3 / motorPower;

//            pl *= scalingfactor;
//            pr *= scalingfactor;

            leftMotor.setPower(pl);
            rightMotor.setPower(pr);


            telemetry.update();
        } while(Math.abs(current_heading - target_heading) > degreeDiffThreshold);

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    void driveUntilDistEqualsStraight(double motorPower, double targetDistance) {
        while(getDistance() == Double.MAX_VALUE); //wait until the sensor takes at least 50 readings

        double initial_heading = 0;


        try {
            initial_heading = angles.heading;
        } catch(Exception e){}


        double distanceThreshold = .02;

        if(targetDistance < getDistance()) {
            motorPower = -Math.abs(motorPower); //drive backwards!
        } else if(targetDistance > getDistance()) {
            motorPower = Math.abs(motorPower); //drive forwards!
        }

        while (Math.abs(getDistance() - targetDistance) > distanceThreshold) {

            double current_heading = initial_heading;

            try {
                current_heading = normalizeDegrees(angles.heading);
            } catch (Exception e) {}

            double error = (current_heading - initial_heading) * drive_straight_p_error_const;


            pl = motorPower - error;
            pr = motorPower + error;

            pl = scale(pl);
            pr = scale(pr);


            leftMotor.setPower(pl);
            rightMotor.setPower(pr);
            telemetry.update();

        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    void driveTicksStraight(double motorPower, int ticks) {

        int initialLeftPosition = leftMotor.getCurrentPosition();
        int initialRightPosition = rightMotor.getCurrentPosition();

        double initial_heading = 0;

        try {
            initial_heading = angles.heading;
        } catch(Exception e){}


        while (Math.abs(initialLeftPosition - leftMotor.getCurrentPosition()) < ticks
                || Math.abs(initialRightPosition - rightMotor.getCurrentPosition()) < ticks) {

            double current_heading = initial_heading;

            try {
                current_heading = normalizeDegrees(angles.heading);
            } catch (Exception e) {}

            double error = (current_heading - initial_heading) * drive_straight_p_error_const;


            pl = motorPower - error;
            pr = motorPower + error;

            pl = scale(pl);
            pr = scale(pr);


            leftMotor.setPower(pl);
            rightMotor.setPower(pr);
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    double angle360(double angle) {
        return (angle > 0) ? angle : 360 + angle;
    }

    double scale(double d) {
        if (d > 1) return 1.0;
        if (d < -1) return -1.0;
        else return d;
    }

    void composeDashboard()
    {
        // The default dashboard update rate is a little too slow for our taste here, so we update faster
        telemetry.setUpdateIntervalMs(200);

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation();
                position = imu.getPosition();

                // The rest of this is pretty cheap to acquire, but we may as well do it
                // all while we're gathering the above.
                loopCycles = getLoopCount();
                i2cCycles = ((II2cDeviceClientUser) imu).getI2cDeviceClient().getI2cCycleCount();
                ms = elapsed.time() * 1000.0;
            }
        });
        telemetry.addLine(
                telemetry.item("loop count: ", new IFunc<Object>() {
                    public Object value() {
                        return loopCycles;
                    }
                }),
                telemetry.item("i2c cycle count: ", new IFunc<Object>() {
                    public Object value() {
                        return i2cCycles;
                    }
                }));

        telemetry.addLine(
                telemetry.item("loop rate: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return formatRate(ms / loopCycles);
                    }
                }),
                telemetry.item("i2c cycle rate: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return formatRate(ms / i2cCycles);
                    }
                }));

        telemetry.addLine(
                telemetry.item("status: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return decodeStatus(imu.getSystemStatus());
                    }
                }),
                telemetry.item("calib: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return decodeCalibration(imu.read8(IBNO055IMU.REGISTER.CALIB_STAT));
                    }
                }));

        telemetry.addLine(
                telemetry.item("heading: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return formatAngle(angle360(angles.heading));
                    }
                }),
                telemetry.item("roll: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return formatAngle(angles.roll);
                    }
                }),
                telemetry.item("pitch: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return formatAngle(angles.pitch);
                    }
                }));
        telemetry.addLine(
                telemetry.item("dist ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return "" + getDistance();
                    }
                })
        );


    }

    String formatAngle(double angle)
    {
        return parameters.angleUnit ==IBNO055IMU.ANGLEUNIT.DEGREES ? formatDegrees(angle) : formatRadians(angle);
    }
    String formatRadians(double radians)
    {
        return formatDegrees(degreesFromRadians(radians));
    }
    String formatDegrees(double degrees)
    {
        return String.format("%.1f", normalizeDegrees(degrees));
    }
    String formatRate(double cyclesPerSecond)
    {
        return String.format("%.2f", cyclesPerSecond);
    }
    String formatPosition(double coordinate)
    {
        String unit = parameters.accelUnit == IBNO055IMU.ACCELUNIT.METERS_PERSEC_PERSEC
                ? "m" : "??";
        return String.format("%.2f%s", coordinate, unit);
    }

    //----------------------------------------------------------------------------------------------
    // Utility
    //----------------------------------------------------------------------------------------------

    /** Normalize the angle into the range [-180,180) */
    double normalizeDegrees(double degrees)
    {
        while (degrees >= 180.0) degrees -= 360.0;
        while (degrees < -180.0) degrees += 360.0;
        return degrees;
    }
    double degreesFromRadians(double radians)
    {
        return radians * 180.0 / Math.PI;
    }

    /** Turn a system status into something that's reasonable to show in telemetry */
    String decodeStatus(int status)
    {
        switch (status)
        {
            case 0: return "idle";
            case 1: return "syserr";
            case 2: return "periph";
            case 3: return "sysinit";
            case 4: return "selftest";
            case 5: return "fusion";
            case 6: return "running";
        }
        return "unk";
    }

    /** Turn a calibration code into something that is reasonable to show in telemetry */
    String decodeCalibration(int status)
    {
        StringBuilder result = new StringBuilder();

        result.append(String.format("s%d", (status >> 2) & 0x03));  // SYS calibration status
        result.append(" ");
        result.append(String.format("g%d", (status >> 2) & 0x03));  // GYR calibration status
        result.append(" ");
        result.append(String.format("a%d", (status >> 2) & 0x03));  // ACC calibration status
        result.append(" ");
        result.append(String.format("m%d", (status >> 0) & 0x03));  // MAG calibration status

        return result.toString();
    }
}

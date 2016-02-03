package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Nathaniel on 1/15/16
 */

public abstract class BaseAutoNonLin extends OpMode {


    void setup() {
        setupElectronics();

        /* gyro initialization */
        startHeading = 0;

        long systemTime = System.nanoTime();
        prevTime = systemTime;
        try {
            gyro = new AdafruitIMU(hardwareMap, "bno055"
                    , (byte) (AdafruitIMU.BNO055_ADDRESS_A * 2)//By convention the FTC SDK always does 8-bit I2C bus
                    , (byte) AdafruitIMU.OPERATION_MODE_IMU);
        } catch (RobotCoreException e) {
            Log.i("FtcRobotController", "Exception: " + e.getMessage());
        }

        systemTime = System.nanoTime();
        gyro.startIMU();//Set up the IMU as needed for a continual stream of I2C reads.
        telemetry.addData("FtcRobotController", "IMU Start method finished in: "
                + (-(systemTime - (systemTime = System.nanoTime()))) + " ns.");

        Thread headingThread = new Thread() {
            public void run() {
                while (true) {
                    updateHeadingThreaded();
                }
            }
        };

        headingThread.start();
    }




    DcMotor encoderMotor1;
    DcMotor encoderMotor2;
    DcMotor left;
    DcMotor right;

    void setupElectronics() {
        left = encoderMotor1 = hardwareMap.dcMotor.get("left");
        right = encoderMotor2 = hardwareMap.dcMotor.get("right");

        left.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.REVERSE);
    }

    void updateGlobalHeadingAndEncodeInformation() {
        updateMotorEncoderStart();
        updateStartHeading();
    }

    void updateMotorEncoderStart() {
        if(startEncoderUpdateState != state) {
            start = encoderMotor2.getCurrentPosition();
            start2 = encoderMotor2.getCurrentPosition();
            startEncoderUpdateState = state;
        }
    }

    void updateStartHeading() {
        if(startHeadingUpdateState != state) {
            startHeading = curHeading;
            startHeadingUpdateState = state;
        }
    }

    void incrementState() {
        state++;
    }

    void incrementStateAndStopMotors() {
        incrementState();
        stopMotors();
    }


    public static void updateHeadingThreaded() {
        long systemTime = System.nanoTime();
        long elapsedTime = systemTime - prevTime;
        prevTime = systemTime;
        double elapsedSeconds = elapsedTime / 1000000000;
        systemTime = System.nanoTime();

        //Update gyro values
        gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
    }


    //drive encoders values
    int start;
    int start2;

    //state variable
    int state = 0;
    int startHeadingUpdateState = -1;
    int startEncoderUpdateState = -1;

    ColorSensor colorSensor1;
    ColorSensor colorSensor2;

    final int LINE_THRESHOLD = 20;

    //gyroscope
    static long prevTime = System.nanoTime();

    static AdafruitIMU gyro;
    double startHeading = 0;
    //The following arrays contain both the Euler angles reported by the IMU (indices = 0) AND the
    // Tait-Bryan angles calculated from the 4 components of the quaternion vector (indices = 1)
    static volatile double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];

    static double curHeading;
    static double prevHeading;

    public void updateIMU() {
        gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
    }



    abstract AtomicUtil.Alliance getTeam();

    /**
     * Drive the left side of the robot.
     *
     * @param power from -1.0 to 1.0
     */
    abstract void moveLeft(double power);

    /**
     * Drive the right side of the robot.
     *
     * @param power from -1.0 to 1.0
     */
    abstract void moveRight(double power);

    /**
     * Stop the motors.
     */
    void stopMotors() {
        moveLeft(0);
        moveRight(0);
    }

    /**
     * Drive forward, moving both sides of the robot.
     *
     * @param power from -1.0 to 1.0
     */
    void drive(double power) {
        moveLeft(power);
        moveRight(power);
    }

    /**
     * Rotate the robot.
     *
     * @param power double from 0.0.to 1.0
     * @param dir Direction CLOCKWISE or COUNTERCLOCKWISE
     */
    void rotate(double power, AtomicUtil.Direction dir) {
        int d;
        switch (dir) {
            case CLOCKWISE:
                d = 1;
                break;
            case COUNTERCLOCKWISE:
                d = -1;
                break;
            default:
                d = 0;
                break;
        }
        moveLeft(-power * d);
        moveRight(power * d);
    }

    void rotateTicks(double power, AtomicUtil.Direction dir, int ticks) {
        while(Math.abs(encoderMotor1.getCurrentPosition() - start) < ticks)
            rotate(power, dir);
        stopMotors();
    }

    void rotateDegs(double power, AtomicUtil.Direction dir, double deg) {
        if (dir == AtomicUtil.Direction.CLOCKWISE)
            if(curHeading > (360 - deg))
                rotate(power, dir);
        else if (dir == AtomicUtil.Direction.COUNTERCLOCKWISE)
            if (curHeading < deg)
                rotate(power, dir);


        //need stop motors code
    }

    /**
     * Drive forward until encoder tick threshold is met.
     * @param power
     * @param ticks Number of encoder ticks to travel
     */
    void dumbTicks(double power, int ticks) {
        if (Math.abs(encoderMotor1.getCurrentPosition() - start) < ticks) {
            drive(power);
        } else {
            incrementStateAndStopMotors();
        }
    }

    /**
     * Drive forward until encoder tick threshold is met.
     * @param power
     * @param ticks Number of encoder ticks to travel
     */
    public void driveTicksStraight(double power, int ticks) {
        double error_const = .04;

        if (Math.abs(encoderMotor1.getCurrentPosition() - start) < ticks
                || Math.abs(encoderMotor2.getCurrentPosition() - start2) < ticks) { //might need to use && instead of ||
            double pl = power;
            double pr = power;


            double error = curHeading - startHeading;

            pl-=error * error_const;
            pr+=error * error_const;

            pl = scale(pl);
            pr = scale(pr);

            moveLeft(pl);
            moveRight(pr);
            telemetry.addData("m1:", Math.abs(encoderMotor1.getCurrentPosition() - start) - ticks);
            telemetry.addData("m2:", Math.abs(encoderMotor2.getCurrentPosition() - start2) - ticks);
        } else {
            incrementStateAndStopMotors();
        }
    }


    /**
     * Ensure a number is within bounds for motor controllers.
     * @param d number
     * @return number, as long as it's within -1 and 1. Returns the bound otherwise.
     */
    double scale(double d) {
        if (d > 1) return 1.0;
        if (d < -1) return -1.0;
        else return d;
    }

    /**
     * Determine if a number is positive or negative
     * @param d number
     * @return 1 if number is positive, -1 if it's negative
     */
    double sign(double d) {
        if (d >= 0) return 1.0;
        else return -1.0;
    }


}

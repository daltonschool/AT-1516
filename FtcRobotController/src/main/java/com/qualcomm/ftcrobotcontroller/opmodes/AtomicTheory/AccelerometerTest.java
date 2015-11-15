package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import android.hardware.SensorManager;
import android.hardware.Sensor;
import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;
/**
 * Created by davis on 11/14/15.
 */
public class AccelerometerTest extends OpMode{
  FtcRobotControllerActivity app;
  public void init() {
    app = ((FtcRobotControllerActivity)hardwareMap.appContext);

  }

  public void loop() {
    telemetry.addData("AccX", app.ax);
    telemetry.addData("AccY", app.ay);
    telemetry.addData("AccZ", app.az);
  }

}

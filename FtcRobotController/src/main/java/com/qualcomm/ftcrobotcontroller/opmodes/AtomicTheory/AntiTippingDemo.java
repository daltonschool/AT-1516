package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

import android.app.Activity;

import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;

/**
 * Created by davis on 11/18/15.
 */
public class AntiTippingDemo extends AlphaDrive{
  FtcRobotControllerActivity app;
  double goal;
  double k;
  public void init() {
    super.init();
    app = ((FtcRobotControllerActivity)hardwareMap.appContext);
    goal = getPosition();
    k = .1; // change this for more precision.
  }
  public void loop() {
    double p = scale_motor_power((goal - getPosition())*k);
    drive(p);
  }

  public double getPosition() {
    double m = 1;
    if (app.az < 0) m = -1;
    double pos = app.ax*m;
    return pos;
  }
}

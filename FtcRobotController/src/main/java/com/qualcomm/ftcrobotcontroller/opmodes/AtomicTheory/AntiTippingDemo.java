package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

import android.app.Activity;

import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;

/**
 * Created by davis on 11/18/15.
 */
public class AntiTippingDemo extends AlphaDrive{
  FtcRobotControllerActivity app;
  float goal;
  double k;
  public void init() {
    super.init();
    app = ((FtcRobotControllerActivity)hardwareMap.appContext);
    goal = app.ax;
    k = .1;
  }
  public void loop() {
    double p = scale_motor_power((goal - app.ax)*k);
    moveLeft(p);
    moveRight(p);
  }
}

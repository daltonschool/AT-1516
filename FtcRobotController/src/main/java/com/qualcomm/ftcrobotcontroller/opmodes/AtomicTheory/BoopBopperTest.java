package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

import android.graphics.Color;

/**
 * Created by davis on 10/2/15.
 */
public class BoopBopperTest extends BaseLinearOpMode {
  public void runOpMode() throws InterruptedException{
    config();
    waitOneFullHardwareCycle();

    bopper.setPosition(.5);

    waitForStart();

    float hsvValues[] = {0F,0F,0F};

    while (opModeIsActive()) {
      if (colorSensor1.red() > colorSensor2.red())
        bopper.setPosition(0);
      else
        bopper.setPosition(1);

      telemetry.addData("Left sensor blue", colorSensor1.blue());
      telemetry.addData("Left sensor red", colorSensor1.red());
      telemetry.addData("Right sensor blue", colorSensor2.blue());
      telemetry.addData("Right sensor red", colorSensor2.red());

      waitOneFullHardwareCycle();
    }
  }
}

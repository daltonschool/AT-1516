//package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory.Archived;
//
//import com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory.AtomicUtil;
//import com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory.LinearAlpha;
//
///**
// * Created by davis on 12/28/15.
// */
//public class ColorSensorTest extends LinearAlpha {
//  public void runOpMode() throws InterruptedException{
//    setup();
//    waitForStart();
//    colorSensor1.enableLed(true);
//    colorSensor2.enableLed(true);
//    while(opModeIsActive()) {
//      telemetry.addData("Alpha 1", colorSensor1.alpha());
//      telemetry.addData("Alpha 2", colorSensor2.alpha());
//    }
//  }
//
//  AtomicUtil.Alliance getTeam() {
//    return null;
//  }
//}

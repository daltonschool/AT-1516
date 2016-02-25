//package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;
//
///**
// * Created by nathaniel on 2/2/16.
// */
//public class TurningTestNonLin extends BaseAutoNonLin {
//
//    AtomicUtil.Alliance getTeam() {
//        return AtomicUtil.Alliance.RED;
//    }
//
//    public void init() {
//        setup();
//
//    }
//
//    public void loop() {
//        switch(state) {
//            case 0: {
//                updateGlobalHeadingAndEncodeInformation();
//
//                int tickstoDrive = 1500; //obviously this values aren't proven at all
//
//                driveTicksStraight(.5, tickstoDrive); //updates state automagically
//
//                state = 1;
//
//                break;
//            }
//
//            case 1: {
//                stopMotors();
//
//                break;
//            }
//        }
//
//    }
//
//}

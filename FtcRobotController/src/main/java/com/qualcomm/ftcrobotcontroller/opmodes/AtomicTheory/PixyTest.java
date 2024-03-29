package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * Created by davis on 11/7/15.
 */
public class PixyTest extends LinearOpMode{

  DeviceInterfaceModule dim;
  public void runOpMode() throws InterruptedException{
    dim = hardwareMap.deviceInterfaceModule.get("dim");
    waitForStart();

    short lastSync = 0x0000;
    while(opModeIsActive()) {
      byte[] data = dim.getCopyOfReadBuffer(0);
      telemetry.addData("Bytes", "Value");
      for (int i = 0; i < data.length; i += 2) {
        telemetry.addData(i + ", " + i+1, bytesToWord(data[i], data[i+1]));
        long t = System.currentTimeMillis();
        while (System.currentTimeMillis() - t < 1000)
          ;
      }
    }

  }

  /**
   * Convert bytes to an word (Little Endian)
   * @param b1 least-significant byte
   * @param b2 most-significant byte
   * @return little-endian word.
   */
  public short bytesToWord(byte b1, byte b2) {
    ByteBuffer b = ByteBuffer.allocateDirect(2);
    b.order(ByteOrder.LITTLE_ENDIAN);
    b.put(b1);
    b.put(b2);
    b.flip();
    return b.getShort();
  }
}

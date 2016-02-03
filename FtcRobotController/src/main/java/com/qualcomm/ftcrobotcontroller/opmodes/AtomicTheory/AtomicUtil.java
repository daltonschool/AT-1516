package com.qualcomm.ftcrobotcontroller.opmodes.AtomicTheory;

/**
 * Created by davis on 10/12/15.
 */
public class AtomicUtil {
  public enum Alliance {
    RED, BLUE;
  }

  public enum Direction {
    FORWARD, BACKWARD, LEFT, RIGHT, CLOCKWISE, COUNTERCLOCKWISE
  }

  public Direction getOtherClockDirection(Direction dir) {
      return (dir == Direction.CLOCKWISE) ? Direction.COUNTERCLOCKWISE : Direction.CLOCKWISE;
  }

}

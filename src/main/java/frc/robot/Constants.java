// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {

    public static final int kDriverControllerPort = 0;

  }

  public static class ElevatorConstants {

    // All of the CAN IDs for the elevator motors
    public static final int kElevator1CANID = 14;
    public static final int kElevator2CANID = 5;
    public static final int kElevator3CANID = 4;
    public static final int kElevator4CANID = 15;

    public static final int kCandiCANID = 55;

    public static final double kPulleyGearRatio = 1.6925;

  }

  public static class WindmillConstants {

    public static final int kWindmillCANID = 6;

    public static final int kWindmillEncoderCANID = 6;

  }

  public static class ManipulatorConstants {

    public static final int kManipulatorCANID = 40;

  }

  /** Constants for the LEDs */
  public static final class LEDConstants {
    /** The CAN ID for the CANdle */
    public static final int kCANdleID = 30;

    /** LED Strip class to define start and length */
    public static class LEDStrip {
      public final int start;
      public final int end;
      public final int length;

      public LEDStrip(int start, int length) {
        this.start = start;
        this.length = length;
        this.end = start + length;
      }
    }

    /** Total number of RGB LEDs. */
    public static final LEDStrip kRGBCANdle = new LEDStrip(0, 8);
    public static final LEDStrip kRGBSection1 = new LEDStrip(8, 11);
    public static final LEDStrip kRGBSection2 = new LEDStrip(19, 10);
    public static final LEDStrip kRGBSection3 = new LEDStrip(30, 11);
    public static final int kRGBCount = kRGBCANdle.length + kRGBSection1.length + kRGBSection2.length
        + kRGBSection3.length; // 40

  }
}

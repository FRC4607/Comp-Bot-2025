// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
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
  public static class WindmillConstants{

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
    /** Total number of RGB LEDs. */
    public static final int kRGBCANdleCount = 8;
    public static final int kRGBSection1Count = 80;
    public static final int kRGBSection2Count = 60; // Eight onboard LEDs too
    public static final int kRGBSection3Count = 80; // Eight onboard LEDs too
    public static final int kRGBCount = kRGBCANdleCount + kRGBSection1Count + kRGBSection2Count + kRGBSection3Count ; 
}
}

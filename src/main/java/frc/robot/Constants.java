// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.stream.Stream;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import static edu.wpi.first.math.util.Units.inchesToMeters;
import static java.util.stream.Collectors.toUnmodifiableList;

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

  public static class FieldConstants {

    public static final Transform2d RELATIVE_SCORING_POSE = new Transform2d(
        inchesToMeters(-25),
        inchesToMeters(6),
        Rotation2d.fromDegrees(-90));

    public static final List<Pose2d> REEF_BRANCH_POSES_BLUE = Stream
        .of(
            new Pose2d(4.347746, 3.467, Rotation2d.fromDegrees(60)), // D
              new Pose2d(4.062584, 3.630770, Rotation2d.fromDegrees(60)), // C
              new Pose2d(3.942648, 3.840490, Rotation2d.fromDegrees(0)), // B
              new Pose2d(3.942648, 4.169106, Rotation2d.fromDegrees(0)), // A
              new Pose2d(4.062584, 4.398912, Rotation2d.fromDegrees(-60)), // L
              new Pose2d(4.347175, 4.515, Rotation2d.fromDegrees(-60)), // K
              new Pose2d(4.588763, 4.542161, Rotation2d.fromDegrees(-120)), // J
              new Pose2d(4.873926, 4.378820, Rotation2d.fromDegrees(-120)), // P
              new Pose2d(4.98, 4.215, Rotation2d.fromDegrees(180)), // H
              new Pose2d(4.994328, 3.841097, Rotation2d.fromDegrees(180)), // G
              new Pose2d(4.95, 3.76, Rotation2d.fromDegrees(120)), // F
              new Pose2d(4.589334, 3.466500, Rotation2d.fromDegrees(120)))// E
        .map(reefPose -> reefPose.plus(RELATIVE_SCORING_POSE))
        .collect(toUnmodifiableList());

    public static final List<Pose2d> REEF_BRANCH_POSES_RED = Stream
        .of(
            new Pose2d(13.200254, 4.585000, Rotation2d.fromDegrees(-120)), // D
              new Pose2d(13.485416, 4.421230, Rotation2d.fromDegrees(-120)), // C
              new Pose2d(13.605352, 4.211510, Rotation2d.fromDegrees(-180)), // B
              new Pose2d(13.605352, 3.882894, Rotation2d.fromDegrees(-180)), // A
              new Pose2d(13.485416, 3.653088, Rotation2d.fromDegrees(120)), // L
              new Pose2d(13.200825, 3.537000, Rotation2d.fromDegrees(120)), // K
              new Pose2d(12.959237, 3.509839, Rotation2d.fromDegrees(60)), // J
              new Pose2d(12.674074, 3.673180, Rotation2d.fromDegrees(60)), // P
              new Pose2d(12.568000, 3.837000, Rotation2d.fromDegrees(0)), // H
              new Pose2d(12.553672, 4.210903, Rotation2d.fromDegrees(0)), // G
              new Pose2d(12.598000, 4.292000, Rotation2d.fromDegrees(-60)), // F
              new Pose2d(12.958666, 4.585500, Rotation2d.fromDegrees(-60)))// E
        .map(reefPose -> reefPose.plus(RELATIVE_SCORING_POSE))
        .collect(toUnmodifiableList());
  }
}

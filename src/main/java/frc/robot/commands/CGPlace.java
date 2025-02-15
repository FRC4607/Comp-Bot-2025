// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Calibrations;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WindmillSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CGPlace extends SequentialCommandGroup {

  /** Creates a new Placement Command group. */
  public CGPlace(double elevatorHeight, double windmillSetpoint, WindmillSubsystem windmill, ElevatorSubsystem elevator) {
    super(
      new ConditionalCommand(
        new InstantCommand(), 
        new ConditionalCommand(
          new InstantCommand(), 
          new Retract(windmill, elevator), 
          () -> windmill.getWindmillSetpoint() < 170 && windmill.getWindmillSetpoint() > 10 && windmill.isAtPosition()), 
        () -> Math.abs(windmillSetpoint - windmill.getWindmillSetpoint()) < 45),
      new SetElevatorSetpoint(elevatorHeight, Calibrations.PlacementCalibrations.kElevatorTolerance, elevator),
      new SetWindmillSetpoint(windmillSetpoint, Calibrations.PlacementCalibrations.kWindmillTolerance, windmill)
    );

  }
}

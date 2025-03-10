// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WindmillSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Retract extends SequentialCommandGroup {

  /** Creates a new Retract Command Group, which lolipops the arm and moves the elevator all the way down. */
  public Retract(WindmillSubsystem windmill, ElevatorSubsystem elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(
      new SetWindmillSetpoint(90, 25, false, false, elevator, windmill),
      new SetElevatorSetpoint(-0.3, 1, false, elevator, windmill)
    );
  }
}

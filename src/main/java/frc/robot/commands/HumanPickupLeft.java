// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WindmillSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HumanPickupLeft extends SequentialCommandGroup {

  /** Creates a new HumanPickupLeft. */
  public HumanPickupLeft(WindmillSubsystem windmill, ElevatorSubsystem elevator) {
    super(
      new Retract(windmill, elevator),
      new SetElevatorSetpoint(24, 10, elevator),
      new SetWindmillSetpoint(225, 5, windmill)
    );
  }
}

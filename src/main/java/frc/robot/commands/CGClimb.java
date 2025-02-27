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
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.WindmillSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CGClimb extends SequentialCommandGroup {

  /** Creates a new Placement Command group, which will lolipop the arm if necessary, then go to the desired elevator height and arm angle.*/
  public CGClimb(WindmillSubsystem windmill, ElevatorSubsystem elevator) {
    super(
      new SetElevatorSetpoint(18, 0.5, true, elevator, windmill).withTimeout(1),
      new SetWindmillSetpoint(180, 5, true, false, elevator, windmill).withTimeout(1),
      new SetElevatorSetpoint(-0.2, 3, true, elevator, windmill)      

    );
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WindmillSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RetractDown extends SequentialCommandGroup {
  /** Creates a new RetractDown, which puts the elevator to a safe passthrough height, then puts the arm in a straight down (pendulum) position. */
  public RetractDown(WindmillSubsystem windmill, ElevatorSubsystem elevator) {
    super(
      new ConditionalCommand(
        new ConditionalCommand(
          new SetWindmillSetpoint(60, 15, false, elevator, windmill), 
          new SetWindmillSetpoint(120, 15, false, elevator, windmill), 
          () -> windmill.getPosition() >= 90
          ),
        new InstantCommand(),  
      () -> windmill.getPosition() < 170 && windmill.getPosition() > 10
      ),
      new SetElevatorSetpoint(25, 2, false, elevator, windmill),
      new SetWindmillSetpoint(270, 5, false, elevator, windmill)
    );
  }
}

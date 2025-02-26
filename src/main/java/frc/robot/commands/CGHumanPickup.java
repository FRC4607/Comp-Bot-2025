// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.WindmillSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CGHumanPickup extends SequentialCommandGroup {

  /** Creates a new HumanPickupLeft. */
  public CGHumanPickup(double windmillSetpoint, double elevatorSetpoint, WindmillSubsystem windmill, ElevatorSubsystem elevator, ManipulatorSubsystem manipulator) {
    super(
      new ParallelCommandGroup(
      new SetWindmillSetpoint(180, 10, false, false, elevator, windmill),
      // new ConditionalCommand(
      //   new InstantCommand(), 
      //   new RetractDown(windmill, elevator), 
      //   () -> elevator.getPosition() > 25),
      new SetElevatorSetpoint(elevatorSetpoint, 10, false, elevator, windmill)
      ),
      new SetWindmillSetpoint(windmillSetpoint, 15, false, true, elevator, windmill).withTimeout(1),
      //new SetManipulatorSpeed(() -> -1.0, manipulator, windmill).withTimeout(0.1),
      new Intake(manipulator, windmill)//,
      //new SetManipulatorSpeed(() -> 1.0, manipulator, windmill).withTimeout(0.05)
    );
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElevatorSetpoint extends Command {

  private ElevatorSubsystem m_elevator;

  private double m_newElevatorSetpoint;

  /** Creates a new SetElevatorSetpoint. */
  public SetElevatorSetpoint(double newElevatorSetpoint, ElevatorSubsystem elevator) {
    m_elevator = elevator;
    m_newElevatorSetpoint = newElevatorSetpoint;

    addRequirements(m_elevator);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_elevator.setElevatorSetpoint(m_newElevatorSetpoint);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

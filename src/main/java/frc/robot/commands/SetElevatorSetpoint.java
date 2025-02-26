// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WindmillSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElevatorSetpoint extends Command {

  private ElevatorSubsystem m_elevator;
  private WindmillSubsystem m_windmill;

  private double m_newElevatorSetpoint;
  private double m_tolerance;
  private boolean m_isClimbing;

  /** Creates a new SetElevatorSetpoint. */
  public SetElevatorSetpoint(double newElevatorSetpoint, double tolerance, boolean isClimbing, ElevatorSubsystem elevator, WindmillSubsystem windmill) {
    m_elevator = elevator;
    m_windmill = windmill;
    m_newElevatorSetpoint = newElevatorSetpoint;
    m_tolerance = tolerance;
    m_isClimbing = isClimbing;

    addRequirements(m_elevator);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Elevator Setpoint Changed To: " + m_newElevatorSetpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_elevator.setElevatorSetpoint(m_newElevatorSetpoint, m_isClimbing, m_windmill);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Elevator Reached Setpoint");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_elevator.getPosition() - m_newElevatorSetpoint) < m_tolerance;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WindmillSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetWindmillSetpoint extends Command {
  /** Creates a new SetWindmillSetpoint. */

  private WindmillSubsystem m_windmill;
  private ElevatorSubsystem m_elevator;

  private boolean m_isClimbing;
  private boolean m_goLongWay;

  private double m_newWindmillSetpoint;
  private double m_tolerance;

  public SetWindmillSetpoint(double newWindmillSetpoint, double tolerance, boolean isClimbing, boolean goLongWay, ElevatorSubsystem elevator, WindmillSubsystem windmill) {
    m_elevator = elevator;
    m_windmill = windmill;
    m_newWindmillSetpoint = newWindmillSetpoint;
    m_tolerance = tolerance;
    m_isClimbing = isClimbing;
    m_goLongWay = goLongWay;

    addRequirements(m_windmill);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("windmill setpoint changed to: " + m_newWindmillSetpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_windmill.setWindmillSetpoint(m_newWindmillSetpoint, m_isClimbing, m_goLongWay, m_elevator);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Windmill Reached Setpoint");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_windmill.getPosition() - m_newWindmillSetpoint) < m_tolerance;
    // TODO: Move tolerance to constants file
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WindmillSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Retract extends Command {
  WindmillSubsystem m_windmill;
  private double m_newWindmillSetpoint;

  /** Creates a new Retract. */
  public Retract(WindmillSubsystem windmill) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_windmill = windmill;

    addRequirements(m_windmill);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_windmill.setWindmillSetpoint(90, false);
    // TODO: Move the newWindmillSetpoint to Calibrations
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Retract running");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Retract finished");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (Math.abs(m_windmill.getPosition() - m_windmill.getWindmillSetpoint()) < 5) {
    //   // TODO: Move the setpoint (90) and the tolerance (5)
    //   return true;
    // } else {
    //   return false;
    return true;
  }
}

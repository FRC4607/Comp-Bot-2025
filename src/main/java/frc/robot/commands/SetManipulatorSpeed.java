// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetManipulatorSpeed extends Command {

  private ManipulatorSubsystem m_manipulator;
  private DoubleSupplier m_newVelocity;

  /** Creates a new SetManipulatorSpeed. */
  public SetManipulatorSpeed(DoubleSupplier newVelocity, ManipulatorSubsystem manipulator) {
    
    m_manipulator = manipulator;
    m_newVelocity = newVelocity;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_manipulator);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_manipulator.setVelocity(0.5 * Math.copySign(m_newVelocity.getAsDouble() * m_newVelocity.getAsDouble(), m_newVelocity.getAsDouble()));
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

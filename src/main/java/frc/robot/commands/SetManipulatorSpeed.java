// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.Calibrations;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.WindmillSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetManipulatorSpeed extends Command {

  private ManipulatorSubsystem m_manipulator;
  private WindmillSubsystem m_windmill;
  private DoubleSupplier m_newVelocity;

  /** Creates a new SetManipulatorSpeed. */
  public SetManipulatorSpeed(DoubleSupplier newVelocity, ManipulatorSubsystem manipulator, WindmillSubsystem windmill) {
    
    m_manipulator = manipulator;
    m_windmill = windmill;
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
    
    if (m_windmill.getPosition() > 80 && m_windmill.getPosition() < 100) {
      m_manipulator.setVelocity(-Math.copySign(m_newVelocity.getAsDouble() * m_newVelocity.getAsDouble() * Calibrations.ManipulatorCalibrations.kManipulatorMaxSpeed, m_newVelocity.getAsDouble()));
    } else {
      if (m_windmill.isLeft()) {
        if (m_windmill.isAbove()) {
          // negative if in the upper left quadrant
          m_manipulator.setVelocity(Math.copySign(m_newVelocity.getAsDouble() * m_newVelocity.getAsDouble() * Calibrations.ManipulatorCalibrations.kManipulatorMaxSpeed, m_newVelocity.getAsDouble()));
        } else {
          // positive if in the bottom left quadrant
          m_manipulator.setVelocity(Math.copySign(m_newVelocity.getAsDouble() * m_newVelocity.getAsDouble() * Calibrations.ManipulatorCalibrations.kManipulatorMaxSpeed, m_newVelocity.getAsDouble()));
        }
      } else {
        if (m_windmill.isAbove()) {
          // positive if in the upper right quadrant
          m_manipulator.setVelocity(-Math.copySign(m_newVelocity.getAsDouble() * m_newVelocity.getAsDouble() * Calibrations.ManipulatorCalibrations.kManipulatorMaxSpeed, m_newVelocity.getAsDouble()));
        } else {
          // negative if in the bottom right quadrant
          m_manipulator.setVelocity(-Math.copySign(m_newVelocity.getAsDouble() * m_newVelocity.getAsDouble() * Calibrations.ManipulatorCalibrations.kManipulatorMaxSpeed, m_newVelocity.getAsDouble()));
        }   
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_manipulator.setVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

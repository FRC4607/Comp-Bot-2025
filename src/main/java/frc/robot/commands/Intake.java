// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Calibrations;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.WindmillSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Intake extends Command {

  private ManipulatorSubsystem m_manipulator;
  private WindmillSubsystem m_windmill;

  private LinearFilter filter = LinearFilter.movingAverage(10);

  private double[] inputBuffer = {30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0};
  private double[] outputBuffer = {};

  /** Creates a new Intake. */
  public Intake(ManipulatorSubsystem manipulator, WindmillSubsystem windmill) {
    m_manipulator = manipulator;
    m_windmill = windmill;


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_manipulator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_manipulator.setVelocity(Calibrations.ManipulatorCalibrations.kManipulatorMaxSpeed);
    filter.reset(inputBuffer, outputBuffer);

    System.out.println("Manipulator picking up");
    new SetManipulatorSpeed(() -> 1.0, m_manipulator, m_windmill);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Manipulator is picking up");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_manipulator.setVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    var deltaCurrent = m_manipulator.getStatorCurrent() - filter.lastValue();
    filter.calculate(m_manipulator.getStatorCurrent());
    
    return deltaCurrent > Calibrations.ManipulatorCalibrations.kCurrentThreshold;
  }
}

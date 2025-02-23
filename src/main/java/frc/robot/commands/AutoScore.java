// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.concurrent.ScheduledExecutorService;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.WindmillSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoScore extends Command {
  private CommandSwerveDrivetrain m_drivetrain;
  private WindmillSubsystem m_windmill;
  private ElevatorSubsystem m_elevator;
  private ManipulatorSubsystem m_manipulator;

  private Supplier<Pose2d> m_robotPose;
  private Pose2d m_targetPose;

  private boolean isRed;
  private double m_windmillSetpoint;
  private double m_elevatorSetpoint;
  /** Creates a new AutoScore. */
  public AutoScore(Supplier<Pose2d> robotPose, double windmillSetpoint, double elevatorSetpoint, CommandSwerveDrivetrain drivetrain, WindmillSubsystem windmill, ElevatorSubsystem elevator, ManipulatorSubsystem manipulator) {
    m_robotPose = robotPose;
    m_drivetrain = drivetrain;
    m_windmill = windmill;
    m_elevator = elevator;
    m_manipulator = manipulator;

    m_windmillSetpoint = windmillSetpoint;
    m_elevatorSetpoint = elevatorSetpoint;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("AutoScore Initialized");
    isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

    m_targetPose = m_robotPose.get().nearest(isRed ? Constants.FieldConstants.REEF_BRANCH_POSES_RED : Constants.FieldConstants.REEF_BRANCH_POSES_BLUE);

    new SequentialCommandGroup(
      new AutoDriveToPosition(m_targetPose, m_robotPose, m_drivetrain),
      new CGPlace(m_elevatorSetpoint, m_windmillSetpoint, m_windmill, m_elevator),
      new SetManipulatorSpeed(() -> 1, m_manipulator, m_windmill)
    ).schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

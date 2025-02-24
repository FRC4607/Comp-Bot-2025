// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Calibrations;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoDriveToPosition extends Command {

  private CommandSwerveDrivetrain m_drivetrain;

  private Pose2d m_targetPose;
  private Supplier<Pose2d> m_robotPose;

  private double m_xSpeed;
  private double m_ySpeed;
  private double m_rotationSpeed;

  //private static final Distance m_translationTolerance = Calibrations.AutoDriveToPositionCalibrations.kAutoAlignTranslationTolerance;
  //private static final Angle m_rotationTolerance = Calibrations.AutoDriveToPositionCalibrations.kAutoAlignRotationTolerance;

  private final ProfiledPIDController m_xController;
  private final ProfiledPIDController m_yController;
  private final ProfiledPIDController m_rotationController;

  private static final TrapezoidProfile.Constraints m_translationConstraints = new TrapezoidProfile.Constraints(
    Calibrations.AutoDriveToPositionCalibrations.kTranslationMaxVelocity.in(MetersPerSecond), 
    Calibrations.AutoDriveToPositionCalibrations.kTranslationMaxAcceleration.in(MetersPerSecondPerSecond));

  private static final TrapezoidProfile.Constraints m_rotationConstraints = new TrapezoidProfile.Constraints(
    Calibrations.AutoDriveToPositionCalibrations.kRotationMaxVelocity, 
    Calibrations.AutoDriveToPositionCalibrations.kRotationMaxAcceleration);

  private final FieldCentric m_swerveRequest = new FieldCentric()
    .withSteerRequestType(SteerRequestType.MotionMagicExpo)
    .withDriveRequestType(DriveRequestType.Velocity)
    .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
  /** Creates a new AutoDriveToPosition. */
  public AutoDriveToPosition(Pose2d targetPose, Supplier<Pose2d> robotPose, CommandSwerveDrivetrain drivetrain) {
    m_drivetrain = drivetrain;
    m_targetPose = targetPose;
    m_robotPose = robotPose;

    m_xController = new ProfiledPIDController(
      Calibrations.AutoDriveToPositionCalibrations.kXkP, 
      Calibrations.AutoDriveToPositionCalibrations.kXkI, 
      Calibrations.AutoDriveToPositionCalibrations.kXkD, 
      m_translationConstraints);
    
    m_xController.setTolerance(Calibrations.AutoDriveToPositionCalibrations.kAutoAlignTranslationTolerance);

    m_yController = new ProfiledPIDController(
      Calibrations.AutoDriveToPositionCalibrations.kYkP, 
      Calibrations.AutoDriveToPositionCalibrations.kYkI, 
      Calibrations.AutoDriveToPositionCalibrations.kYkD, 
      m_translationConstraints);

    m_yController.setTolerance(Calibrations.AutoDriveToPositionCalibrations.kAutoAlignTranslationTolerance);

    m_rotationController = new ProfiledPIDController(
      Calibrations.AutoDriveToPositionCalibrations.kRotationkP, 
      Calibrations.AutoDriveToPositionCalibrations.kRotationkI, 
      Calibrations.AutoDriveToPositionCalibrations.kRotationkD, 
      m_rotationConstraints);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("AutoDrive Initiated");
    var robotPose = m_robotPose.get();
    m_xController.reset(robotPose.getX());
    m_yController.reset(robotPose.getY());
    m_rotationController.reset(robotPose.getRotation().getRadians());

    m_xController.setGoal(m_targetPose.getX());
    m_yController.setGoal(m_targetPose.getY());
    m_rotationController.setGoal(m_targetPose.getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Autodrive in progress");
    var robotPose = m_robotPose.get();
    
    m_xSpeed = m_xController.atGoal() ? 0 : m_xController.calculate(robotPose.getX());
    
    m_ySpeed = m_yController.atGoal() ? 0 : m_yController.calculate(robotPose.getY());

    m_rotationSpeed = m_rotationController.atGoal() ? 0 : m_rotationController.calculate(robotPose.getRotation().getRadians());

    m_drivetrain.setControl(m_swerveRequest.withVelocityX(m_xSpeed).withVelocityY(m_ySpeed).withRotationalRate(m_rotationSpeed));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Autodrive finished");
    m_drivetrain.setControl(new SwerveRequest.Idle());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_xController.atGoal() && m_yController.atGoal() && m_rotationController.atGoal();
  }
}

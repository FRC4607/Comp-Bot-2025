// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ApplyConfigs;
import frc.robot.commands.CGHumanPickup;
import frc.robot.commands.CGPlace;
import frc.robot.commands.Retract;
import frc.robot.commands.SetElevatorSetpoint;
import frc.robot.commands.SetManipulatorSpeed;
import frc.robot.commands.SetWindmillSetpoint;
import frc.robot.commands.SetWindmillSpeed;
import frc.robot.commands.setElevatorSpeed;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.WindmillSubsystem;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
    private final WindmillSubsystem m_windmill = new WindmillSubsystem();
    private final ManipulatorSubsystem m_manipulator = new ManipulatorSubsystem();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Basic elevator controls
        //joystick.povDown().onTrue(new setElevatorSpeed(-0.05, m_elevator));
        //joystick.povDown().onFalse(new setElevatorSpeed(0, m_elevator));
        //joystick.povUp().onTrue(new setElevatorSpeed(0.05, m_elevator));
        //joystick.povUp().onFalse(new setElevatorSpeed(0, m_elevator));

        // joystick.povUp().onTrue(new SetElevatorSetpoint(20, m_elevator));
        // joystick.povDown().onTrue(new SetElevatorSetpoint(3.0, m_elevator));
        //joystick.povLeft().onTrue(new SetElevatorSetpoint(-0.5, m_elevator));

        // joystick.povUp().onTrue(new SetWindmillSetpoint(180, m_windmill));
        // joystick.povDown().onTrue(new SetWindmillSetpoint(0, m_windmill));
        // joystick.povRight().onTrue(new SetWindmillSetpoint(90, m_windmill));
        // joystick.povLeft().onTrue(new SetWindmillSetpoint(-90, m_windmill));

        // joystick.povRight().onTrue(new SequentialCommandGroup(new Retract(m_windmill), new SetWindmillSetpoint(0, m_windmill)));
        // joystick.povLeft().onTrue(new SequentialCommandGroup(new Retract(m_windmill), new SetWindmillSetpoint(180, m_windmill)));
        // joystick.povLeft().onTrue(new Retract(m_windmill));

        joystick.a().and(joystick.leftBumper()).onTrue(new CGHumanPickup(225, m_windmill, m_elevator));
        joystick.povUp().and(joystick.leftBumper()).onTrue(new CGPlace(30, 45, m_windmill, m_elevator));
        joystick.povLeft().and(joystick.leftBumper()).onTrue(new CGPlace(20, 45, m_windmill, m_elevator));
        joystick.povRight().and(joystick.leftBumper()).onTrue(new CGPlace(10, 45, m_windmill, m_elevator));
        joystick.povDown().and(joystick.leftBumper()).onTrue(new CGPlace(10, 45, m_windmill, m_elevator));

        joystick.a().and(joystick.rightBumper()).onTrue(new CGHumanPickup(-45, m_windmill, m_elevator));
        joystick.povUp().and(joystick.rightBumper()).onTrue(new CGPlace(30, 135, m_windmill, m_elevator));
        joystick.povLeft().and(joystick.rightBumper()).onTrue(new CGPlace(20, 135, m_windmill, m_elevator));
        joystick.povRight().and(joystick.rightBumper()).onTrue(new CGPlace(10, 135, m_windmill, m_elevator));
        joystick.povDown().and(joystick.rightBumper()).onTrue(new CGPlace(10, 135, m_windmill, m_elevator));


        // joystick.leftTrigger(0.1).onTrue(new SetManipulatorSpeed(0.2, m_manipulator)).onFalse(new SetManipulatorSpeed(0.0, m_manipulator));
        // joystick.rightTrigger(0.1).onTrue(new SetManipulatorSpeed(-0.2, m_manipulator)).onFalse(new SetManipulatorSpeed(0.0, m_manipulator));

        m_manipulator.setDefaultCommand(new SetManipulatorSpeed(() -> joystick.getLeftTriggerAxis() - joystick.getRightTriggerAxis(), m_manipulator));
    
                SmartDashboard.putData("Apply Config", new ApplyConfigs(m_windmill, m_elevator));
        
                drivetrain.registerTelemetry(logger::telemeterize);
            }
        
            public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}

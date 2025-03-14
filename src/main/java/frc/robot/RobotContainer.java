// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ApplyConfigs;
import frc.robot.commands.AutoScore;
import frc.robot.commands.CGClimb;
import frc.robot.commands.CGHumanPickup;
import frc.robot.commands.CGPlace;
import frc.robot.commands.CGPlaceWithOuttake;
import frc.robot.commands.Retract;
import frc.robot.commands.RetractDown;
import frc.robot.commands.SetElevatorSetpoint;
import frc.robot.commands.SetManipulatorSpeed;
import frc.robot.commands.SetWindmillSetpoint;
import frc.robot.commands.SetWindmillSpeed;
import frc.robot.commands.SwitchLimelightPipelines;
import frc.robot.commands.setElevatorSpeed;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
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
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
    public final WindmillSubsystem m_windmill = new WindmillSubsystem();
    public final ManipulatorSubsystem m_manipulator = new ManipulatorSubsystem();
    private final LEDSubsystem m_leds = new LEDSubsystem();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        
        configureBindings();
        
        // NamedCommands.registerCommand("PlaceL1Right", new CGPlace(0, 105, m_windmill, m_elevator).withTimeout(2));
        NamedCommands.registerCommand("PlaceL4Left", new CGPlace(52, 35, m_windmill, m_elevator).withTimeout(2));
        NamedCommands.registerCommand("Outtake Piece", new SetManipulatorSpeed(() -> 1, m_manipulator, m_windmill).withTimeout(0.25));
        NamedCommands.registerCommand("Outtake Piece Reverse", new SetManipulatorSpeed(() -> -1, m_manipulator, m_windmill).withTimeout(0.25));
        NamedCommands.registerCommand("Retract", new Retract(m_windmill, m_elevator).withTimeout(2));
        NamedCommands.registerCommand("RetractDown", new RetractDown(m_windmill, m_elevator).withTimeout(2));
        NamedCommands.registerCommand("Intake Piece", new CGHumanPickup(-69, 36.92, m_windmill, m_elevator, m_manipulator).withTimeout(3));
        NamedCommands.registerCommand("Turn Off Limelight", new SwitchLimelightPipelines(9));
        NamedCommands.registerCommand("Turn On Limelight", new SwitchLimelightPipelines(0));
        NamedCommands.registerCommand("Align", new AutoScore(() -> drivetrain.getState().Pose, 9.5, 135, drivetrain, m_windmill, m_elevator, m_manipulator));
        NamedCommands.registerCommand("PlaceL4Right", new CGPlace(52, 145, m_windmill, m_elevator).withTimeout(2));

        // new EventTrigger("PlaceL4Left").onTrue(new CGPlaceWithOuttake(52, 35, () -> 1, m_windmill, m_elevator, m_manipulator).withTimeout(2));
        // new EventTrigger("PlaceL4LeftReverse").onTrue(new CGPlaceWithOuttake(52, 35, () -> -1, m_windmill, m_elevator, m_manipulator).withTimeout(2));
        // new EventTrigger("PlaceL4RightReverse").onTrue(new CGPlaceWithOuttake(52, 145, () -> -1, m_windmill, m_elevator, m_manipulator).withTimeout(2));
        // new EventTrigger("Outtake Piece").onTrue(new SetManipulatorSpeed(() -> 1, m_manipulator, m_windmill));
        // new EventTrigger("Intake Piece").onTrue(new CGHumanPickup(-60, 34.72, m_windmill, m_elevator, m_manipulator));
        // new EventTrigger("Retract").onTrue((new Retract(m_windmill, m_elevator)));
        


        autoChooser = AutoBuilder.buildAutoChooser("3m test path");
        SmartDashboard.putData("Auto Mode", autoChooser);
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

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // joystick.povUp().onTrue(new SetWindmillSetpoint(90, 0, m_elevator, m_windmill));
        // joystick.povDown().onTrue(new SetElevatorSetpoint(3, 0, m_elevator, m_windmill));

        // reset the field-centric heading on left bumper press
        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Setpoints for when the robot is on the left side of the reef.
        joystick.povUp().and(joystick.leftBumper()).onTrue(new CGPlace(52.5, 35, m_windmill, m_elevator));
        joystick.povLeft().and(joystick.leftBumper()).onTrue(new CGPlace(24, 45, m_windmill, m_elevator));
        joystick.povRight().and(joystick.leftBumper()).onTrue(new CGPlace(9.5, 45, m_windmill, m_elevator));
        joystick.povDown().and(joystick.leftBumper()).onTrue(new CGPlace(0, 45, m_windmill, m_elevator));

        // // Setpoints for when the robot is on the right side of the reef.
        joystick.povUp().and(joystick.leftBumper().negate()).onTrue(new CGPlace(52.5, 145, m_windmill, m_elevator));
        joystick.povLeft().and(joystick.leftBumper().negate()).onTrue(new CGPlace(24, 135, m_windmill, m_elevator));
        joystick.povRight().and(joystick.leftBumper().negate()).onTrue(new CGPlace(9.5, 135, m_windmill, m_elevator));
        joystick.povDown().and(joystick.leftBumper().negate()).onTrue(new CGPlace(0, 135, m_windmill, m_elevator));

        joystick.back().onTrue(new CGPlace(52.5, 90, m_windmill, m_elevator));
        
        //joystick.povRight().whileTrue(new AutoScore(() -> drivetrain.getState().Pose, 9.5, 135, drivetrain, m_windmill, m_elevator, m_manipulator));
        
            
        joystick.rightBumper().onTrue(new CGHumanPickup(-69, 36.92, m_windmill, m_elevator, m_manipulator).withTimeout(4));
        joystick.a().onTrue(new CGPlace(0, 10, m_windmill, m_elevator));

        joystick.b().onTrue(new Retract(m_windmill, m_elevator));
        joystick.x().onTrue(new RetractDown(m_windmill, m_elevator));
        joystick.y().onTrue(new CGPlace(4, 120, m_windmill, m_elevator)).onFalse(new CGClimb(m_windmill, m_elevator));

        m_manipulator.setDefaultCommand(new SetManipulatorSpeed(() -> (-joystick.getRightTriggerAxis() + joystick.getLeftTriggerAxis()), m_manipulator, m_windmill));
    
                // SmartDashboard.putData("Apply Config", new ApplyConfigs(m_windmill, m_elevator));
        
                drivetrain.registerTelemetry(logger::telemeterize);
        
        // SmartDashboard.putData("Disable Logger", new InstantCommand(() -> { SignalLogger.stop(); System.out.println("Logs Stopped");}));


    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

}
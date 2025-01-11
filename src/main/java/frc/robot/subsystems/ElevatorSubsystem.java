// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  // Creates the TalonFX motors for the elevator.
  private final TalonFX m_elevator1;
  private final TalonFX m_elevator2;
  private final TalonFX m_elevator3;
  private final TalonFX m_elevator4;

  private final Follower m_follower;
  private final Follower m_followerInv;
  private final MotionMagicTorqueCurrentFOC m_motionMagicTorqueCurrentFOC;

  public ElevatorSubsystem() {

    // Initializes the TalonFX motors for the elevator.
    m_elevator1 = new TalonFX(0);
    m_elevator2 = new TalonFX(0);
    m_elevator3 = new TalonFX(0);
    m_elevator4 = new TalonFX(0);
    //TODO: Move CAN IDs to constants folder

    m_motionMagicTorqueCurrentFOC = new MotionMagicTorqueCurrentFOC(0);

    // Creates a configurator for the motors in this subsystem.
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Feedforward and PID settings for the motors in this subsystem.
    config.Slot0.kG = 0;
    config.Slot0.kS = 0;
    config.Slot0.kV = 0;
    config.Slot0.kA = 0;
    config.Slot0.kP = 0;
    config.Slot0.kD = 0;
    //TODO: Move these values to Calibrations file.
    
    // Applies the configs to all the motors in this subsystem.
    m_elevator1.getConfigurator().apply(config.Slot0);  
    m_elevator2.getConfigurator().apply(config.Slot0);
    m_elevator3.getConfigurator().apply(config.Slot0);
    m_elevator4.getConfigurator().apply(config.Slot0);

    // Declares elevator1 as lead motor. Other motors are set to follow.
    m_follower = new Follower(0, false);
    m_followerInv = new Follower(0, true);
    //TODO: Add elevator1 CAN ID from constants here
    m_elevator2.setControl(m_follower);
    m_elevator3.setControl(m_follower);
    m_elevator4.setControl(m_follower);
    //TODO: Set motors to follow followerInv if they need to be reversed
  }

  /**
   * Passes in a value in degrees for the Motion Magic Motion Profiler to use.
   * 
   * @param newElevatorSetpoint - New setpoint for the elevator in inches.
   */
  public void setElevatorSetpoint(double newElevatorSetpoint) {

    m_elevator1.setControl(m_motionMagicTorqueCurrentFOC.withPosition(newElevatorSetpoint * 1.6925));
    //TODO: move that number to a Constants file
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

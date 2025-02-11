// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ManipulatorSubsystem extends SubsystemBase {

  private final TalonFX m_motor;

  private final DutyCycleOut m_request;

  /** Creates a new ManupulatorSubsystem. */
  public ManipulatorSubsystem() {

    m_motor = new TalonFX(43, "kachow");

    m_request = new DutyCycleOut(0.0);
    
    TalonFXConfiguration config = new TalonFXConfiguration();

    m_motor.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setVelocity (double newManipulatorVelocity) {

    m_motor.set(newManipulatorVelocity);
  }
}

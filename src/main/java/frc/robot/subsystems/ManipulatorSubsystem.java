// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations;

public class ManipulatorSubsystem extends SubsystemBase {

  private final TalonFX m_motor;

  private final DutyCycleOut m_request;

  private final VelocityTorqueCurrentFOC m_velocityRequest;

  /** Creates a new ManupulatorSubsystem. */
  public ManipulatorSubsystem() {

    m_motor = new TalonFX(13, "kachow");

    m_request = new DutyCycleOut(0.0);

    m_velocityRequest = new VelocityTorqueCurrentFOC(0);
    
    TalonFXConfiguration config = new TalonFXConfiguration();

    Slot0Configs slot0 = config.Slot0;

    slot0.kS = Calibrations.ManipulatorCalibrations.kManipulatorKS;
    slot0.kV = Calibrations.ManipulatorCalibrations.kManipulatorKV;
    slot0.kA = Calibrations.ManipulatorCalibrations.kManipulatorKA;
    slot0.kP = Calibrations.ManipulatorCalibrations.kManipulatorKP;
    slot0.kD = Calibrations.ManipulatorCalibrations.kManipulatorKD;

    config.MotionMagic.MotionMagicAcceleration = Calibrations.ManipulatorCalibrations.kManipulatormaxAcceleration;

    config.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(100))
    .withPeakReverseTorqueCurrent(Amps.of(-100));
    m_motor.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Sets the velocity of the Manipulator (rpms) using MotionMagic VelocityTorqueCurrentFOC
   * 
   * @param newManipulatorVelocity
   */
  public void setVelocity (double newManipulatorVelocity) {

    m_motor.setControl(m_velocityRequest.withVelocity(newManipulatorVelocity));
    System.out.println("Velocity changed to: " + newManipulatorVelocity);
  }
}

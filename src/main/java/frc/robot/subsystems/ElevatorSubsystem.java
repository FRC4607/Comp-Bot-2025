// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.CANdiConfigurator;
import com.ctre.phoenix6.configs.DigitalInputsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S1FloatStateValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  // Creates the TalonFX motors for the elevator.
  private final TalonFX m_elevator1;
  private final TalonFX m_elevator2;
  private final TalonFX m_elevator3;
  private final TalonFX m_elevator4; 

  private final CANdi m_CaNdi;

  // Creates two followers for running the motors in sync.
  private final Follower m_follower;
  private final Follower m_followerInv;

  // 

  // Creates the class for the motion profiler.
  private final MotionMagicTorqueCurrentFOC m_motionMagicTorqueCurrentFOC;

  public ElevatorSubsystem() {

    // Initializes the TalonFX motors for the elevator.
    m_elevator1 = new TalonFX(Constants.ElevatorConstants.kElevator1CANID, "kachow");
    m_elevator2 = new TalonFX(Constants.ElevatorConstants.kElevator2CANID, "kachow");
    m_elevator3 = new TalonFX(Constants.ElevatorConstants.kElevator3CANID, "kachow");
    m_elevator4 = new TalonFX(Constants.ElevatorConstants.kElevator4CANID, "kachow");

    // initialize the CANdi
    m_CaNdi = new CANdi(25, "kachow");
    // TODO: move deviceid to Constants

    // initializes the motion magic motion profiler.
    m_motionMagicTorqueCurrentFOC = new MotionMagicTorqueCurrentFOC(0);
    
    // Creates a new limitconfigs
    HardwareLimitSwitchConfigs limitConfigs = new HardwareLimitSwitchConfigs();

    // Creates a CANdi Configurator
    CANdiConfiguration candiConfig = new CANdiConfiguration();

    // Creates a Configurator for a soft limit.
    SoftwareLimitSwitchConfigs softLimitConfigs = new SoftwareLimitSwitchConfigs();

    // Creates a configurator for the motors in this subsystem.
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Gravity type for this subsystem.
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    
    // Feedforward and PID settings for the motors in this subsystem.
    config.Slot0.kG = Calibrations.ElevatorCalibrations.kElevatorkG;
    config.Slot0.kS = Calibrations.ElevatorCalibrations.kElevatorkS;
    config.Slot0.kV = Calibrations.ElevatorCalibrations.kElevatorkV;
    config.Slot0.kA = Calibrations.ElevatorCalibrations.kElevatorkA;
    config.Slot0.kP = Calibrations.ElevatorCalibrations.kElevatorkP;
    config.Slot0.kD = Calibrations.ElevatorCalibrations.kElevatorkD;

    // Configs to be used by the MotionMagicConfigs class
    config.MotionMagic.MotionMagicCruiseVelocity = Calibrations.ElevatorCalibrations.kMaxSpeedMotionMagic;
    config.MotionMagic.MotionMagicAcceleration = Calibrations.ElevatorCalibrations.kMaxAccelerationMotionMagic;
    config.TorqueCurrent.PeakForwardTorqueCurrent = Calibrations.ElevatorCalibrations.kMaxElevatorCurrentPerMotor;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -Calibrations.ElevatorCalibrations.kMaxElevatorCurrentPerMotor;

    // Configures all of the limit settings for the CANdi.
    limitConfigs.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANdiS1;
    limitConfigs.ReverseLimitRemoteSensorID = m_CaNdi.getDeviceID();
    limitConfigs.ReverseLimitEnable = true;
    limitConfigs.ReverseLimitAutosetPositionEnable = true;
    limitConfigs.ReverseLimitAutosetPositionValue = 0.0;

    // Configures all of the soft limit settings on the elevator1 motor
    softLimitConfigs.ForwardSoftLimitEnable = true;
    softLimitConfigs.ForwardSoftLimitThreshold = 88.3;
    
    // Applies CANdi and Soft limits to the elevator1 motor.
    m_elevator1.getConfigurator().apply(limitConfigs);
    m_elevator1.getConfigurator().apply(softLimitConfigs);
    
    // Configures the CANdi Closed (tripped) and float (open) states. These settings can vary based on the type of sensor.
    candiConfig.DigitalInputs.S1CloseState = S1CloseStateValue.CloseWhenLow;
    candiConfig.DigitalInputs.S1FloatState = S1FloatStateValue.PullHigh;
    m_CaNdi.getConfigurator().apply(candiConfig);
    
    // Applies the configs to all the motors in this subsystem.
    m_elevator1.getConfigurator().apply(config.Slot0);  
    m_elevator2.getConfigurator().apply(config.Slot0);
    m_elevator3.getConfigurator().apply(config.Slot0);
    m_elevator4.getConfigurator().apply(config.Slot0);

    // Sets the neutral mode of all of the elevator motors to Brake Mode.
    m_elevator1.setNeutralMode(NeutralModeValue.Brake);
    m_elevator2.setNeutralMode(NeutralModeValue.Brake);
    m_elevator3.setNeutralMode(NeutralModeValue.Brake);
    m_elevator4.setNeutralMode(NeutralModeValue.Brake);

    // Declares elevator1 as lead motor. Other motors are set to follow.
    m_follower = new Follower(Constants.ElevatorConstants.kElevator1CANID, false);
    m_followerInv = new Follower(Constants.ElevatorConstants.kElevator1CANID, true);

    // Setting the follower mode that each motor will follow.
    m_elevator2.setControl(m_followerInv);
    m_elevator3.setControl(m_follower);
    m_elevator4.setControl(m_followerInv);
  }

  /**
   * Passes in a value in degrees for the Motion Magic Motion Profiler to use.
   * 
   * @param newElevatorSetpoint - New setpoint for the elevator in inches.
   */
  public void setElevatorSetpoint(double newElevatorSetpoint) {

    //Sets the setpoint of elevator1 motor using the MotionMagic Motion Profiler.
    m_elevator1.setControl(m_motionMagicTorqueCurrentFOC.withPosition(newElevatorSetpoint * Constants.ElevatorConstants.kPulleyGearRatio));

  }

  /**
   * Passes in a value for manual control of the elevator as velocity.
   * 
   * @param newElevatorVelocity - Motor output from a scale of 0 to 1.
   */
  public void setElevatorVelocity(double newElevatorVelocity) {

    // Sets the elevator velocity on a scale from 0 to 1.
    m_elevator1.set(newElevatorVelocity);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Candy Bar", m_CaNdi.getS1Closed().getValue().booleanValue());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

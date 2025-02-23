// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.CANdiConfigurator;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.DigitalInputsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S1FloatStateValue;
import com.ctre.phoenix6.signals.S2CloseStateValue;
import com.ctre.phoenix6.signals.S2FloatStateValue;

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

  private boolean m_pastCaNdi;

  // Creates two followers for running the motors in sync.
  private final Follower m_follower;
  private final Follower m_followerInv;

  private TalonFXConfiguration m_config;

  private final DynamicMotionMagicTorqueCurrentFOC m_request;

  public ElevatorSubsystem() {

    // Initializes the TalonFX motors for the elevator.
    m_elevator1 = new TalonFX(Constants.ElevatorConstants.kElevator1CANID, "kachow");
    m_elevator2 = new TalonFX(Constants.ElevatorConstants.kElevator2CANID, "kachow");
    m_elevator3 = new TalonFX(Constants.ElevatorConstants.kElevator3CANID, "kachow");
    m_elevator4 = new TalonFX(Constants.ElevatorConstants.kElevator4CANID, "kachow");

    // initialize the CANdi
    m_CaNdi = new CANdi(Constants.ElevatorConstants.kCandiCANID, "kachow");

    // Creates a CANdi Configurator
    CANdiConfiguration candiConfig = new CANdiConfiguration();

    final DynamicMotionMagicTorqueCurrentFOC request = new DynamicMotionMagicTorqueCurrentFOC(
      0, 
      Calibrations.ElevatorCalibrations.kMaxSpeedMotionMagic, 
      Calibrations.ElevatorCalibrations.kMaxAccelerationMotionMagic, 
      0);

    m_request = request;

    // Creates a configurator for the motors in this subsystem.
    TalonFXConfiguration config = new TalonFXConfiguration();

    Slot0Configs slot0Configs = config.Slot0;

    MotionMagicConfigs magicConfigs = config.MotionMagic;

    TorqueCurrentConfigs currentConfig = config.TorqueCurrent;

    SoftwareLimitSwitchConfigs softLimitConfigs = config.SoftwareLimitSwitch;

    HardwareLimitSwitchConfigs limitConfigs = config.HardwareLimitSwitch;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Gravity type for this subsystem.
    slot0Configs.GravityType = GravityTypeValue.Elevator_Static;
    
    // Feedforward and PID settings for the motors in this subsystem.
    slot0Configs.kG = Calibrations.ElevatorCalibrations.kElevatorkG;
    slot0Configs.kS = Calibrations.ElevatorCalibrations.kElevatorkS;
    slot0Configs.kV = Calibrations.ElevatorCalibrations.kElevatorkV;
    slot0Configs.kA = Calibrations.ElevatorCalibrations.kElevatorkA;
    slot0Configs.kP = Calibrations.ElevatorCalibrations.kElevatorkP;
    slot0Configs.kD = Calibrations.ElevatorCalibrations.kElevatorkD;

    // Configs to be used by the MotionMagicConfigs class
    magicConfigs.MotionMagicCruiseVelocity = Calibrations.ElevatorCalibrations.kMaxSpeedMotionMagic;
    magicConfigs.MotionMagicAcceleration = Calibrations.ElevatorCalibrations.kMaxAccelerationMotionMagic;
    currentConfig.PeakForwardTorqueCurrent = Calibrations.ElevatorCalibrations.kMaxElevatorCurrentPerMotor;
    currentConfig.PeakReverseTorqueCurrent = -Calibrations.ElevatorCalibrations.kMaxElevatorCurrentPerMotor;

    

    // Configures all of the limit settings for the CANdi.
    limitConfigs.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANdiS1;
    limitConfigs.ReverseLimitRemoteSensorID = m_CaNdi.getDeviceID();
    limitConfigs.ReverseLimitEnable = true;
    //limitConfigs.ReverseLimitAutosetPositionEnable = m_CaNdi.getS1Closed().hasUpdated();
    //limitConfigs.ReverseLimitAutosetPositionValue = 0.0;

    // Configures all of the soft limit settings on the elevator1 motor
    softLimitConfigs.ForwardSoftLimitEnable = true;
    softLimitConfigs.ForwardSoftLimitThreshold = 88;
    
    // Configures the CANdi Closed (tripped) and float (open) states. These settings can vary based on the type of sensor.
    candiConfig.DigitalInputs.S1CloseState = S1CloseStateValue.CloseWhenLow;
    candiConfig.DigitalInputs.S1FloatState = S1FloatStateValue.PullHigh;
    candiConfig.DigitalInputs.S2CloseState = S2CloseStateValue.CloseWhenLow;
    candiConfig.DigitalInputs.S2FloatState = S2FloatStateValue.PullHigh;
    m_CaNdi.getConfigurator().apply(candiConfig);
    
    // Applies the configs to all the motors in this subsystem.
    m_elevator1.getConfigurator().apply(config);  
    m_elevator2.getConfigurator().apply(config);
    m_elevator3.getConfigurator().apply(config);
    m_elevator4.getConfigurator().apply(config);

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

    m_config = config;
  }

  /**
   * Passes in a value in degrees for the Motion Magic Motion Profiler to use.
   * 
   * @param newElevatorSetpoint - New setpoint for the elevator in inches.
   */
  public void setElevatorSetpoint(double newElevatorSetpoint, boolean isClimbing, WindmillSubsystem windmill) {

    //Sets the setpoint of elevator1 motor using the MotionMagic Motion Profiler.
    //m_elevator1.setControl(m_motionMagicTorqueCurrentFOC.withPosition(newElevatorSetpoint * Constants.ElevatorConstants.kPulleyGearRatio));
    if (isClimbing) {
      if (
        ((
          windmill.getWindmillSetpoint() < 320 
          && windmill.getWindmillSetpoint() > 210 
        )
        || (
          windmill.getPosition() < 320 
        && windmill.getPosition() > 210
        )) 
        && newElevatorSetpoint < 25
        ) {
          m_elevator1.setControl(m_request.withPosition(25 * Constants.ElevatorConstants.kPulleyGearRatio).withVelocity(12));
          System.out.println("Invalid Elevator Setpoint, automatically set to the safe value of 25 inches");
      } else {
        m_elevator1.setControl(m_request.withPosition(newElevatorSetpoint * Constants.ElevatorConstants.kPulleyGearRatio).withVelocity(12));
        System.out.println("Elevator Setpoint Changed successfully");
      }
    } else {
      if (
        ((
          windmill.getWindmillSetpoint() < 320 
          && windmill.getWindmillSetpoint() > 210 
        )
        || (
          windmill.getPosition() < 320 
        && windmill.getPosition() > 210
        )) 
        && newElevatorSetpoint < 25
        ) {
        System.out.println("Invalid Elevator Setpoint, automatically set to the safe value of 25 inches");
        m_elevator1.setControl(m_request.withPosition(25 * Constants.ElevatorConstants.kPulleyGearRatio).withVelocity(Calibrations.ElevatorCalibrations.kMaxSpeedMotionMagic));
    } else {
      m_elevator1.setControl(m_request.withPosition(newElevatorSetpoint * Constants.ElevatorConstants.kPulleyGearRatio).withVelocity(Calibrations.ElevatorCalibrations.kMaxSpeedMotionMagic));
    }
    }
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
    if ((m_CaNdi.getS2Closed().getValue().booleanValue() != m_pastCaNdi) && (m_pastCaNdi == false)) {
      m_elevator1.setPosition(0);
    }
    m_pastCaNdi = m_CaNdi.getS2Closed().getValue().booleanValue();
    
    SmartDashboard.putBoolean("Candy Bar", m_CaNdi.getS2Closed().getValue().booleanValue());
    SmartDashboard.putNumber("Elevator Position", getPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
   * Applies configs of the elevator motors that may have been changed on the fly.
   */
  public void editConfig() {
  // m_config.Slot0.kS = SmartDashboard.getNumber("elevator kS", Calibrations.ElevatorCalibrations.kElevatorkS);
  // m_config.Slot0.kG = SmartDashboard.getNumber("elevator kG", Calibrations.ElevatorCalibrations.kElevatorkG);
  // m_config.Slot0.kP = SmartDashboard.getNumber("elevator kP", Calibrations.ElevatorCalibrations.kElevatorkP);
  // m_config.Slot0.kD = SmartDashboard.getNumber("elevator kD", Calibrations.ElevatorCalibrations.kElevatorkD);

  // m_elevator1.getConfigurator().apply(m_config);

  }

  /**
   * Gets the position of the elevator in Inches.
   * 
   * @return the position of the elevator.
   */
  public double getPosition() {
    return m_elevator1.getPosition().getValueAsDouble() / Constants.ElevatorConstants.kPulleyGearRatio;
  }

  /**
   * Gets the position of the elevator from a range of 0 to 1, with 0 being stowed and 1 being fully extended.
   * 
   * @return position of the elevator.
   */
  public double getRangeRelativePosition() {
    return m_elevator1.getPosition().getValueAsDouble() / 52;
  }

  /**
   * Gets the sepoint of the arm in inches.
   * 
   * @return The setpoint of the arm in inches.
   */
  public double getSetpoint() {
    return m_request.Position / Constants.ElevatorConstants.kPulleyGearRatio;
  }

}
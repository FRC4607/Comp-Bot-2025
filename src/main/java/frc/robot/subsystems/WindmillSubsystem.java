
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations;
import frc.robot.Constants;

public class WindmillSubsystem extends SubsystemBase{

    // Creates the windmill motor.
    private final TalonFX m_windmotor;

    private TalonFXConfiguration m_config;

    private final CANcoder m_encoder;

    private boolean pastGoLongWay = false;

    // Creates the motion profiler for the arm.
    private final DynamicMotionMagicTorqueCurrentFOC m_request = new DynamicMotionMagicTorqueCurrentFOC(
        0, 
        Calibrations.WindmillCalibrations.kMaxSpeedMotionMagic, 
        Calibrations.WindmillCalibrations.kMaxAccelerationMotionMagic, 
        Calibrations.WindmillCalibrations.kMaxJerkMotionMagic);

    public WindmillSubsystem() {

        // Initializes the motor on the windmill.
        m_windmotor = new TalonFX(Constants.WindmillConstants.kWindmillCANID, "kachow");

        m_encoder = new CANcoder(Constants.WindmillConstants.kWindmillEncoderCANID, "kachow");
        // Initializes the motion profiler.

        m_config = new TalonFXConfiguration();

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

        m_config.ClosedLoopGeneral.ContinuousWrap = true;
        
        encoderConfig.MagnetSensor.MagnetOffset = Calibrations.WindmillCalibrations.kWindmillEncoderOffset;
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.0;
    
        /* Configure gear ratio */
        FeedbackConfigs fdb = m_config.Feedback;
        fdb.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        fdb.FeedbackRemoteSensorID = Constants.WindmillConstants.kWindmillEncoderCANID;
        fdb.SensorToMechanismRatio = 1;
        fdb.RotorToSensorRatio = 74.4;

        /* Configure Motion Magic velocity, Acceleration, and Jerk */
        MotionMagicConfigs mm = m_config.MotionMagic;
        mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(Calibrations.WindmillCalibrations.kMaxSpeedMotionMagic)) // 5 (mechanism) rotations per second cruise
          .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(Calibrations.WindmillCalibrations.kMaxAccelerationMotionMagic)) // Take approximately 0.5 seconds to reach max vel
          // Take approximately 0.1 seconds to reach max accel 
          .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(Calibrations.WindmillCalibrations.kMaxJerkMotionMagic));
    
        // Slot 0 gains for MotionMagic
        Slot0Configs slot0 = m_config.Slot0;
        slot0.kG = Calibrations.WindmillCalibrations.kWindmillkG;
        slot0.kS = Calibrations.WindmillCalibrations.kWindmillkS;
        slot0.kV = Calibrations.WindmillCalibrations.kWindmillkV;
        slot0.kA = Calibrations.WindmillCalibrations.kWindmillkA;
        slot0.kP = Calibrations.WindmillCalibrations.kWindmillkP;
        slot0.kD = Calibrations.WindmillCalibrations.kWindmillkD;

        slot0.GravityType = GravityTypeValue.Arm_Cosine;

        m_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_config.CurrentLimits.StatorCurrentLimit = Calibrations.WindmillCalibrations.kMaxWindmillStatorCurrentPerMotor;
        // config.CurrentLimits.SupplyCurrentLimit = 60;
        m_config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        m_windmotor.getConfigurator().apply(m_config);
        m_encoder.getConfigurator().apply(encoderConfig);
    
    }

   @Override
    public void periodic(){

        // SmartDashboard.putNumber("Windmill", m_windmotor.getSupplyCurrent().getValueAsDouble());

        SmartDashboard.putNumber("Windmill Position", getPosition());
        // SmartDashboard.putNumber("Windmill Encoder Position", getEncoderPosition());
        // SmartDashboard.putNumber("Windmill Setpoint", getWindmillSetpoint());
        // SmartDashboard.putNumber("Raw Windmill Encoder Postion", getRawEncoderPosition());
        
    }

    /**
     * Takes in a value in degrees and sets it as the new setpoint.
     * 
     * @param newWindmillSetpoint The new setpoint in degrees.
     */
    public void setWindmillSetpoint(double newWindmillSetpoint, boolean isClimbing, boolean goLongWay, ElevatorSubsystem elevator) {
        
        // if (goLongWay) {
        //     m_config.ClosedLoopGeneral.ContinuousWrap = false;
        //     m_windmotor.getConfigurator().refresh(m_config.ClosedLoopGeneral);
        // } else {
        //     m_config.ClosedLoopGeneral.ContinuousWrap = true;
        //     m_windmotor.getConfigurator().refresh(m_config.ClosedLoopGeneral);
        // }
        if (isClimbing) {
            if ((elevator.getPosition() < 24 || elevator.getSetpoint() < 24) && newWindmillSetpoint >= 90 && newWindmillSetpoint < 270) {
                m_windmotor.setControl(m_request.withPosition(210 / 360).withVelocity(0.25));
                System.out.println("Invalid Windmill Setpoint, set to the safe value of 210 degrees");
            } else if ((elevator.getPosition() < 24 || elevator.getSetpoint() < 24) && (newWindmillSetpoint >= 270 || newWindmillSetpoint < 90)) {
                m_windmotor.setControl(m_request.withPosition(330 / 360).withVelocity(0.25));
                System.out.println("Invalid Windmill Setpoint, set to the safe value of 210 degrees");
            } else {
                m_windmotor.setControl(m_request.withPosition(newWindmillSetpoint).withVelocity(0.25));
                System.out.println("Windmill Setpoint Set to: " + newWindmillSetpoint);
            }
        } else {
            if ((elevator.getPosition() < 24 || elevator.getSetpoint() < 24) && newWindmillSetpoint >= 90 && newWindmillSetpoint < 270) {
                m_windmotor.setControl(m_request.withPosition(210 / 360).withAcceleration(Calibrations.WindmillCalibrations.kMaxAccelerationMotionMagic).withVelocity(Calibrations.WindmillCalibrations.kMaxSpeedMotionMagic));
                System.out.println("Invalid Windmill Setpoint, set to the safe value of 210 degrees");
            } else if ((elevator.getPosition() < 24 || elevator.getSetpoint() < 24) && (newWindmillSetpoint >= 270 || newWindmillSetpoint < 90)) {
                m_windmotor.setControl(m_request.withPosition(330 / 360).withAcceleration(Calibrations.WindmillCalibrations.kMaxAccelerationMotionMagic).withVelocity(Calibrations.WindmillCalibrations.kMaxSpeedMotionMagic));
                System.out.println("Invalid Windmill Setpoint, set to the safe value of 210 degrees");
            } else {
                m_windmotor.setControl(m_request.withPosition(newWindmillSetpoint).withAcceleration(Calibrations.WindmillCalibrations.kMaxAccelerationMotionMagic).withVelocity(Calibrations.WindmillCalibrations.kMaxSpeedMotionMagic));
                System.out.println("Windmill Setpoint Set to: " + newWindmillSetpoint);
            }
        }

        

        // Sets the setpoint of windmill motor using the MotionMagic Motion Profiler.
        m_windmotor.setControl(m_request.withPosition(newWindmillSetpoint / 360));
    }

    public void setWindmillSpeed(double newSpeed) {
        m_windmotor.set(newSpeed);
    }

    public double getPosition() {
        return (((m_windmotor.getPosition().getValueAsDouble() * 360) % 360) + 360) % 360;
    }
    public double getEncoderPosition() {
        return (((m_encoder.getPosition().getValueAsDouble() * 360) % 360) + 360) % 360;
    }

    public double getRawEncoderPosition() {
        return m_encoder.getPosition().getValueAsDouble() * 360;
    }

    public double getWindmillSetpoint() {
        return (m_request.Position * 360) % 360;
    }

    public boolean isAtPosition () {
        return Math.abs(getPosition() - getWindmillSetpoint()) < Calibrations.WindmillCalibrations.kWindmillTolerance;
    }

    public void applyConfigs() {
        // m_config.Slot0.kG = SmartDashboard.getNumber("windmill kG", Calibrations.WindmillCalibrations.kWindmillkG);
        // m_config.Slot0.kS = SmartDashboard.getNumber("windmill kS", Calibrations.WindmillCalibrations.kWindmillkS);
        // m_config.Slot0.kP = SmartDashboard.getNumber("windmill kP", Calibrations.WindmillCalibrations.kWindmillkP);
        // m_config.Slot0.kD = SmartDashboard.getNumber("windmill kD", Calibrations.WindmillCalibrations.kWindmillkD);

        m_windmotor.getConfigurator().apply(m_config);
    }

    public boolean isLeft() {
        if (getPosition() >= 90 && getPosition() < 270) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isAbove() {
        if (getPosition() >= 0 && getPosition() < 180) {
            return true;
        } else {
            return false;
        }
    }

}

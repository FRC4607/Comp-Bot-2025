
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
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

    

    private final CANcoder m_encoder;

    // Creates the motion profiler for the arm.
    private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    public WindmillSubsystem() {

        // Initializes the motor on the windmill.
        m_windmotor = new TalonFX(Constants.WindmillConstants.kWindmillCANID, "kachow");

        m_encoder = new CANcoder(3, "kachow");
        // Initializes the motion profiler.

        TalonFXConfiguration cfg = new TalonFXConfiguration();

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

        cfg.ClosedLoopGeneral.ContinuousWrap = true;

        encoderConfig.MagnetSensor.MagnetOffset = Calibrations.WindmillCalibrations.kWindmillEncoderOffset;
    
        /* Configure gear ratio */
        FeedbackConfigs fdb = cfg.Feedback;
        fdb.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        fdb.FeedbackRemoteSensorID = 3;
        fdb.SensorToMechanismRatio = 1; // 12.8 rotor rotations per mechanism rotation
        fdb.RotorToSensorRatio = 64.04;

        /* Configure Motion Magic */
        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(0.25)) // 5 (mechanism) rotations per second cruise
          .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10)) // Take approximately 0.5 seconds to reach max vel
          // Take approximately 0.1 seconds to reach max accel 
          .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));
    
        Slot0Configs slot0 = cfg.Slot0;
        slot0.kG = Calibrations.WindmillCalibrations.kWindmillkG;
        slot0.kS = Calibrations.WindmillCalibrations.kWindmillkS; // Add 0.25 V output to overcome static friction
        slot0.kV = Calibrations.WindmillCalibrations.kWindmillkV; // A velocity target of 1 rps results in 0.12 V output
        slot0.kA = Calibrations.WindmillCalibrations.kWindmillkA; // An acceleration of 1 rps/s requires 0.01 V output
        slot0.kP = Calibrations.WindmillCalibrations.kWindmillkP; // A position error of 0.2 rotations results in 12 V output
        slot0.kD = Calibrations.WindmillCalibrations.kWindmillkD; // A velocity error of 1 rps results in 0.5 V output

        slot0.GravityType = GravityTypeValue.Arm_Cosine;

        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        m_windmotor.getConfigurator().apply(cfg);
        m_encoder.getConfigurator().apply(encoderConfig);
    
    }

   @Override
    public void periodic(){

        SmartDashboard.putNumber("Windmill Position", getPosition());
        SmartDashboard.putNumber("Windmill Encoder Position", getEncoderPosition());
        SmartDashboard.putNumber("Windmill Setpoint", getWindmillSetpoint());
        SmartDashboard.putNumber("Raw Windmill Encoder Postion", getRawEncoderPosition());
        
    }

    /**
     * Takes in a value in degrees and sets it as the new setpoint.
     * 
     * @param newWindmillSetpoint The new setpoint in degrees.
     */
    public void setWindmillSetpoint(double newWindmillSetpoint, boolean isClimbing) {

        //Sets the setpoint of windmill motor using the MotionMagic Motion Profiler.
        m_windmotor.setControl(m_request.withPosition(newWindmillSetpoint / 360));
        System.out.println("Setpoint Changed");
    }

    public void setWindmillSpeed(double newSpeed) {
        m_windmotor.set(newSpeed);
    }

    public double getPosition() {
        return (m_windmotor.getPosition().getValueAsDouble() * 360) % 360;
    }
    public double getEncoderPosition() {
        return (m_encoder.getPosition().getValueAsDouble() * 360) % 360;
    }

    public double getRawEncoderPosition() {
        return m_encoder.getPosition().getValueAsDouble() * 360;
    }

    public double getWindmillSetpoint() {
        return (m_request.Position * 360) % 360;
    }
    

}

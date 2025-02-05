
package frc.robot.subsystems;

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
    private final MotionMagicVoltage m_voltage;

    public WindmillSubsystem() {

        // Initializes the motor on the windmill.
        m_windmotor = new TalonFX(Constants.WindmillConstants.kWindmillCANID, "kachow");

        m_encoder = new CANcoder(3, "kachow");
        // Initializes the motion profiler.
        m_voltage = new MotionMagicVoltage(0);
        // TODO: Define Motion Magic Jerk and Starting position in Calibrations

        // Creates the configurator for the motor.
        TalonFXConfiguration config = new TalonFXConfiguration();

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

        Slot0Configs slot0Configs = config.Slot0;

        ClosedLoopGeneralConfigs generalConfig = config.ClosedLoopGeneral;

        // Sets the mode for the kG value to a cosine function (multiplies it by 1 when horizontal, 0 when vertical, and fills in everything in between)
        slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

        // Feedforward and PID settings for this subsystem
        slot0Configs.kG = Calibrations.WindmillCalibrations.kWindmillkG;
        slot0Configs.kS = Calibrations.WindmillCalibrations.kWindmillkS;
        slot0Configs.kV = Calibrations.WindmillCalibrations.kWindmillkV;
        slot0Configs.kA = Calibrations.WindmillCalibrations.kWindmillkA;
        slot0Configs.kP = Calibrations.WindmillCalibrations.kWindmillkP;
        slot0Configs.kD = Calibrations.WindmillCalibrations.kWindmillkD;

        config.ClosedLoopGeneral.ContinuousWrap = true;

        // From standing behind the robot equals Clockwise Positive
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        config.Feedback.FeedbackRemoteSensorID = m_encoder.getDeviceID();
        config.Feedback.RotorToSensorRatio = 63.942;
        config.Feedback.SensorToMechanismRatio = 1;
        //config.Feedback.FeedbackRotorOffset = 0;
        //config.Feedback.VelocityFilterTimeConstant = 0;
        // TODO: Move these values to Constants

        // Configs to be used by the MotionMagicConfigs Class
        //config.TorqueCurrent.PeakForwardTorqueCurrent = Calibrations.WindmillCalibrations.kMaxWindmillCurrentPerMotor;
        //config.TorqueCurrent.PeakReverseTorqueCurrent = -Calibrations.WindmillCalibrations.kMaxWindmillCurrentPerMotor;

        // applies the configs to the windmotor.
        m_windmotor.getConfigurator().apply(config);
        m_encoder.getConfigurator().apply(encoderConfig);
        m_windmotor.setNeutralMode(NeutralModeValue.Brake);
    }

   @Override
    public void periodic(){

        SmartDashboard.putNumber("Windmill Position", getPosition());
        SmartDashboard.putNumber("Windmill Encoder Position", getEncoderPosition());
        
    }

    /**
     * Takes in a value in degrees and sets it as the new setpoint.
     * 
     * @param newWindmillSetpoint The new setpoint in degrees.
     */
    public void setWindmillSetpoint(double newWindmillSetpoint, boolean isClimbing) {

        //Sets the setpoint of windmill motor using the MotionMagic Motion Profiler.
        m_windmotor.setControl(m_voltage.withPosition(newWindmillSetpoint));

    }

    public void setWindmillSpeed(double newSpeed) {
        m_windmotor.set(newSpeed);
    }

    public double getPosition() {
        return m_windmotor.getPosition().getValueAsDouble();
    }
    public double getEncoderPosition() {
        return m_encoder.getPosition().getValueAsDouble();
    }
    

}


package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations;
import frc.robot.Constants;

public class WindmillSubsystem extends SubsystemBase{

    // Creates the windmill motor.
    private final TalonFX m_windmotor;

    // Creates the motion profiler for the arm.
    private final DynamicMotionMagicVoltage m_voltage;

    public WindmillSubsystem() {

        // Initializes the motor on the windmill.
        m_windmotor = new TalonFX(Constants.WindmillConstants.kWindmillCANID);
       
        // Initializes the motion profiler.
        m_voltage = new DynamicMotionMagicVoltage(0, Calibrations.WindmillCalibrations.kMaxSpeedMotionMagic, Calibrations.WindmillCalibrations.kMaxAccelerationMotionMagic, 0);
        // TODO: Define Motion Magic Jerk and Starting position in Calibrations

        // Creates the configurator for the motor.
        TalonFXConfiguration config = new TalonFXConfiguration();

        ClosedLoopGeneralConfigs generalConfig = config.ClosedLoopGeneral;

        FeedbackConfigs feedbackConfig = config.Feedback;

        // Sets the mode for the kG value to a cosine function (multiplies it by 1 when horizontal, 0 when vertical, and fills in everything in between)
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        // Feedforward and PID settings for this subsystem
        config.Slot0.kG = Calibrations.WindmillCalibrations.kWindmillkG;
        config.Slot0.kS = Calibrations.WindmillCalibrations.kWindmillkS;
        config.Slot0.kV = Calibrations.WindmillCalibrations.kWindmillkV;
        config.Slot0.kA = Calibrations.WindmillCalibrations.kWindmillkA;
        config.Slot0.kP = Calibrations.WindmillCalibrations.kWindmillkP;
        config.Slot0.kD = Calibrations.WindmillCalibrations.kWindmillkD;

        generalConfig.ContinuousWrap = true;

        feedbackConfig.FeedbackRemoteSensorID = 0;
        feedbackConfig.FeedbackRotorOffset = 0;
        feedbackConfig.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        feedbackConfig.RotorToSensorRatio = 0;
        feedbackConfig.SensorToMechanismRatio = 1;
        // TODO: Move these values to Constants

        // Configs to be used by the MotionMagicConfigs Class
        config.TorqueCurrent.PeakForwardTorqueCurrent = Calibrations.WindmillCalibrations.kMaxWindmillCurrentPerMotor;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -Calibrations.WindmillCalibrations.kMaxWindmillCurrentPerMotor;

        // applies the configs to the windmotor.
        m_windmotor.getConfigurator().apply(config);

        
    }

   @Override
    public void periodic(){

    }

    /**
     * Takes in a value in degrees and sets it as the new setpoint.
     * 
     * @param newWindmillSetpoint The new setpoint in degrees.
     */
    public void setWindmillSetpoint(double newWindmillSetpoint, boolean isClimbing) {
        
        if (isClimbing == true) {
            m_voltage.Velocity = 1;
            m_voltage.Acceleration = 1;
            m_voltage.Jerk = 0;
            // TODO: Define these values in Calibrations
        } else {
            m_voltage.Velocity = Calibrations.WindmillCalibrations.kMaxSpeedMotionMagic;
            m_voltage.Acceleration = Calibrations.WindmillCalibrations.kMaxAccelerationMotionMagic;
            m_voltage.Jerk = 0;
            // TODO: Define Jerk in Constants
        }

        //Sets the setpoint of windmill motor using the MotionMagic Motion Profiler.
        m_windmotor.setControl(m_voltage.withPosition(newWindmillSetpoint / 360));

      }
}

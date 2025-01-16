package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations;
import frc.robot.Constants;

public class WindmillSubsystem extends SubsystemBase{

    // Creates the windmill motor.
    private final TalonFX m_windmotor;

    // Creates the motion profiler for the arm.
    private final MotionMagicTorqueCurrentFOC m_motionMagicTorqueCurrentFOC;

    public WindmillSubsystem() {

        // Initializes the motor on the windmill.
        m_windmotor = new TalonFX(Constants.WindmillConstants.kWindmillCANID);
       
        // Initializes the motion profiler.
        m_motionMagicTorqueCurrentFOC = new MotionMagicTorqueCurrentFOC(0);

        // Creates the configurator for the motor.
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Sets the mode for the kG value to a cosine function (multiplies it by 1 when horizontal, 0 when vertical, and fills in everything in between)
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        // Feedforward and PID settings for this subsystem
        config.Slot0.kG = Calibrations.WindmillCalibrations.kWindmillkG;
        config.Slot0.kS = Calibrations.WindmillCalibrations.kWindmillkS;
        config.Slot0.kV = Calibrations.WindmillCalibrations.kWindmillkV;
        config.Slot0.kA = Calibrations.WindmillCalibrations.kWindmillkA;
        config.Slot0.kP = Calibrations.WindmillCalibrations.kWindmillkP;
        config.Slot0.kD = Calibrations.WindmillCalibrations.kWindmillkD;

        // Configs to be used by the MotionMagicConfigs Class
        config.MotionMagic.MotionMagicCruiseVelocity = Calibrations.WindmillCalibrations.kMaxSpeedMotionMagic;
        config.MotionMagic.MotionMagicAcceleration = Calibrations.WindmillCalibrations.kMaxAccelerationMotionMagic;
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
    public void setWindmillSetpoint(double newWindmillSetpoint) {

        //Sets the setpoint of windmill motor using the MotionMagic Motion Profiler.
        m_windmotor.setControl(m_motionMagicTorqueCurrentFOC.withPosition(newWindmillSetpoint / 360));

      }
}

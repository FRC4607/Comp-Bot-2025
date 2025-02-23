package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Calibrations {

    public static class ElevatorCalibrations {

        //All of the PID and Feedforward gains for the MotionMagic Motion profiler.
        public static final double kElevatorkG = 5;
        public static final double kElevatorkS = 2.5;
        public static final double kElevatorkA = 0;
        public static final double kElevatorkV = 0;
        public static final double kElevatorkP = 4;
        public static final double kElevatorkD = 0.6;

        // Motion Magic Configs for the MotionMagicConfigs class for the Elevator
        public static final double kMaxSpeedMotionMagic = 500;

        public static final double kMaxAccelerationMotionMagic = 600;
        public static final double kMaxElevatorCurrentPerMotor = 40;


    }
    public static class WindmillCalibrations {
        
        // The encoder offset for the windmill
        public static final double kWindmillEncoderOffset = 0.265869140;

        // All of the PID and Feedforward gains for the MotionMagic Motion profiler.
        public static final double kWindmillkG = 10.6;
        public static final double kWindmillkS = 3.0;
        public static final double kWindmillkV = 0;
        public static final double kWindmillkA = 0;
        public static final double kWindmillkP = 800;
        public static final double kWindmillkD = 200;

        // Motion Magic Configs for the MotionMagicConfigs Class for the Windmill
        public static final double kMaxSpeedMotionMagic = 1;
        public static final double kMaxAccelerationMotionMagic = 4;
        public static final double kMaxJerkMotionMagic = 100;

        public static final double kMaxWindmillStatorCurrentPerMotor = 80;

        // The windmill will report that it is at it's setpoint if it is within this amount of degrees.
        public static final double kWindmillTolerance = 5;
    }
    public static class ManipulatorCalibrations {
        
        public static final double kManipulatorKS = 37;
        public static final double kManipulatorKV = 0;
        public static final double kManipulatorKA = 0;
        public static final double kManipulatorKP = 10;
        public static final double kManipulatorKD = 0;

        public static final double kManipulatormaxAcceleration = 0;

        public static final double kManipulatorMaxSpeed = 20;

        public static final double kManipulatorMaxStatorCurrent = 100;
    }
    public static class DriverCalibrations {

        // Scaling factor for reducing speed when elevator is extended, value is inversely proportional to the extended speed
        // public static final double kExtendedVelocityScalingFactor = 5;

        public static double kRetractWindmillTolerance = 25;
    }
    public static class PlacementCalibrations {

        // Tolerances for the Place Command group
        public static final double kWindmillTolerance = 1;
        public static final double kElevatorTolerance = 5;

        // Value in which the Place Command Group automatically bypasses the retract command if the 
        // new windmill setpoint is within this many degrees of the old one.
        public static final double kWindmillRetractBypassTolerance = 45;
    }
    public static class AutoDriveToPositionCalibrations {

        // tolerances for robot position
        public static final double kAutoAlignTranslationTolerance = 2;
        public static final double kAutoAlignRotationTolerance = 1;

        // gains for the X-Coordinate PID
        public static final double kXkP = 5;
        public static final double kXkI = 0;
        public static final double kXkD = 0;

        // gains for the Y-Coordinate PID
        public static final double kYkP = 5;
        public static final double kYkI = 0;
        public static final double kYkD = 0;

        // gains for the Rotation PID
        public static final double kRotationkP = 5;
        public static final double kRotationkI = 0;
        public static final double kRotationkD = 0;

        // Translation PID constraints (x-pid and y-pid)
        public static final LinearVelocity kTranslationMaxVelocity = MetersPerSecond.of(1);
        public static final LinearAcceleration kTranslationMaxAcceleration = MetersPerSecond.per(Second).of(1);

        // Rotation PID constraints
        public static final double kRotationMaxVelocity = 1;
        public static final double kRotationMaxAcceleration = 1;
    }
}


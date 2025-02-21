package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Calibrations {

    public static class ElevatorCalibrations {

        //All of the PID and Feedforward gains for the MotionMagic Motion profiler.
        public static final double kElevatorkG = 0.19;
        public static final double kElevatorkS = 0.02; // 0.145?
        public static final double kElevatorkV = 0.0;
        public static final double kElevatorkA = 0.0;
        public static final double kElevatorkP = 0.3;
        public static final double kElevatorkD = 0.0;

        // Motion Magic Configs for the MotionMagicConfigs class for the Elevator
        public static final double kMaxSpeedMotionMagic = 60.0;

        public static final double kMaxAccelerationMotionMagic = 300.0;
        public static final double kMaxElevatorCurrentPerMotor = 40;

    }
    public static class WindmillCalibrations {
        
        // The encoder offset for the windmill
        public static final double kWindmillEncoderOffset = -0.439453125;

        // All of the PID and Feedforward gains for the MotionMagic Motion profiler.
        public static final double kWindmillkG = 0.435;
        public static final double kWindmillkS = 0.21;
        public static final double kWindmillkV = 0;
        public static final double kWindmillkA = 0;
        public static final double kWindmillkP = 20;
        public static final double kWindmillkD = 0;

        // Motion Magic Configs for the MotionMagicConfigs Class for the Windmill
        public static final double kMaxSpeedMotionMagic = 0.25;
        public static final double kMaxAccelerationMotionMagic = 0.75;
        public static final double kMaxWindmillCurrentPerMotor = 40;

        // The windmill will report that it is at it's setpoint if it is within this amount of degrees.
        public static final double kWindmillTolerance = 5;
    }
    public static class PlacementCalibrations {

        // Tolerances for the Place Command group
        public static final double kWindmillTolerance = 1;
        public static final double kElevatorTolerance = 5;

        // Value in which the Place Command Group automatically bypasses the retract command if the 
        // new windmill setpoint is within this many degrees of the old one.
        public static final double kWindmillRetractBypassTolerance = 45;
    }
}


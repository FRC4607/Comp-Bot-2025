package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Calibrations {

    public static class ElevatorCalibrations {

        //All of the PID and Feedforward gains for the MotionMagic Motion profiler.
        public static final double kElevatorkG = 0.19;
        public static final double kElevatorkS = 0.02; // 0.145?
        public static final double kElevatorkV = 0.0;
        public static final double kElevatorkA = 0.0;
        public static final double kElevatorkP = 1.5;
        public static final double kElevatorkD = 0.0;

        // Motion Magic Configs for the MotionMagicConfigs class for the Elevator
        public static final double kMaxSpeedMotionMagic = 100;

        public static final double kMaxAccelerationMotionMagic = 400;
        public static final double kMaxElevatorCurrentPerMotor = 40;


    }
    public static class WindmillCalibrations {
        
        // The encoder offset for the windmill
        public static final double kWindmillEncoderOffset = 0.0615234375;

        // All of the PID and Feedforward gains for the MotionMagic Motion profiler.
        public static final double kWindmillkG = 12.25; // 11.0 for TorqueCurrent
        public static final double kWindmillkS = 1.5; // 3.0 for TorqueCurrent
        public static final double kWindmillkV = 0;
        public static final double kWindmillkA = 0;
        public static final double kWindmillkP = 700;
        public static final double kWindmillkD = 120;

        // Motion Magic Configs for the MotionMagicConfigs Class for the Windmill
        public static final double kMaxSpeedMotionMagic = 12;
        public static final double kMaxAccelerationMotionMagic = 2;
        public static final double kMaxJerkMotionMagic = 100;

        public static final double kMaxWindmillStatorCurrentPerMotor = 80;

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


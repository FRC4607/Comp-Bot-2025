package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Calibrations {

    public static class ElevatorCalibrations {

        //All of the PID and Feedforward gains for the MotionMagic Motion profiler.
        public static final double kElevatorkG = 0.02;
        public static final double kElevatorkS = 0.145;
        public static final double kElevatorkV = 0.0;
        public static final double kElevatorkA = 0.0;
        public static final double kElevatorkP = 0.5;
        public static final double kElevatorkD = 0.0;

        // Motion Magic Configs for the MotionMagicConfigs class for the Elevator
        public static final double kMaxSpeedMotionMagic = 18.0;

        public static final double kMaxAccelerationMotionMagic = 18.0;
        public static final double kMaxElevatorCurrentPerMotor = 40;
        // TODO: Update these values when the robot is built

    }
    public static class WindmillCalibrations {

        // All of the PID and Feedforward gains for the MotionMagic Motion profiler.
        public static final double kWindmillkG = 0.435;
        public static final double kWindmillkS = 0.21;
        public static final double kWindmillkV = 0;
        public static final double kWindmillkA = 0;
        public static final double kWindmillkP = 0;
        public static final double kWindmillkD = 0;

        // Motion Magic Configs for the MotionMagicConfigs Class for the Windmill
        public static final double kMaxSpeedMotionMagic = 0.3;
        public static final double kMaxAccelerationMotionMagic = 0.75;
        public static final double kMaxWindmillCurrentPerMotor = 40;
        // TODO: Update these values when the robot is built
    }
}


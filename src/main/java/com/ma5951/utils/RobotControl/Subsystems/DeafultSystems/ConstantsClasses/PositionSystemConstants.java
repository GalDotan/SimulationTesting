
package com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.ConstantsClasses;

import com.ma5951.utils.RobotControl.Utils.Motor;
import com.ma5951.utils.RobotControl.Utils.Sensors.BaseSensor;

public class PositionSystemConstants {

    public Motor[] MOTORS;
    public double GEAR = 1;
    public double POSITION_CONVERSION_FACTOR = 1;
    public double VELOCITY_CONVERSION_FACTOR = 1;
    public boolean IS_MOTION_MAGIC = false;
    public double KP = 0;
    public double KI = 0;
    public double KD = 0;
    public double CRUISE_VELOCITY = 0;
    public double ACCELERATION = 0;
    public double JERK = 0;
    public double FEED_FORWARD_VOLTAGE = 0;
    public double MIN_POSE = 0;
    public double MAX_POSE = 0;
    public double START_POSE = 0;
    public double TOLERANCE = 0;
    public double STATOR_CURRENT_LIMIT = 40;
    public boolean CURRENT_LIMIT_ENABLED = true;
    public double MOTOR_LIMIT_CURRENT = 40;
    public String LOG_PATH = "/Did Not Specifiyed Log Path!";
    public boolean IS_BRAKE = true;
    public double PEAK_FORWARD_VOLTAGE = 12;
    public double PEAK_REVERSE_VOLTAGE = -12;
    public boolean FOC = false;
    public double INERTIA = 0.02;
    @SuppressWarnings("rawtypes")
    public BaseSensor[] SENSORS;

    public PositionSystemConstants(
            Motor[] motors,
            double gear,
            double positionConversionFactor,
            double velocityConversionFactor,
            boolean isMotionMagic,
            double cruiseVelocity,
            double acceleration,
            double jerk,
            double kP,
            double kI,
            double kD,
            double feedForwardVoltage,
            double minPose,
            double maxPose,
            double startPose,
            double tolerance,
            double statorCurrentLimit,
            boolean currentLimitEnabled,
            double motorLimitCurrent,
            String logPath,
            boolean isBrake,
            double peakForwardVoltage,
            double peakReverseVoltage,
            boolean foc,
            double inertia,
            @SuppressWarnings("rawtypes") BaseSensor[] sensors) {
        MOTORS = motors;
        GEAR = gear;
        POSITION_CONVERSION_FACTOR = positionConversionFactor;
        VELOCITY_CONVERSION_FACTOR = velocityConversionFactor;
        IS_MOTION_MAGIC = isMotionMagic;
        CRUISE_VELOCITY = cruiseVelocity;
        ACCELERATION = acceleration;
        JERK = jerk;
        KP = kP;
        KI = kI;
        KD = kD;
        FEED_FORWARD_VOLTAGE = feedForwardVoltage;
        MIN_POSE = minPose;
        MAX_POSE = maxPose;
        START_POSE = startPose;
        TOLERANCE = tolerance;
        STATOR_CURRENT_LIMIT = statorCurrentLimit;
        CURRENT_LIMIT_ENABLED = currentLimitEnabled;
        MOTOR_LIMIT_CURRENT = motorLimitCurrent;
        LOG_PATH = logPath;
        IS_BRAKE = isBrake;
        PEAK_FORWARD_VOLTAGE = peakForwardVoltage;
        PEAK_REVERSE_VOLTAGE = peakReverseVoltage;
        FOC = foc;
        SENSORS = sensors;
        INERTIA = inertia;
    }

    /*
     * Use when utilizing Motion Magic.
     */

    public PositionSystemConstants(
            Motor[] motors,
            double gear,
            double positionConversionFactor,
            double velocityConversionFactor,
            double kP,
            double kI,
            double kD,
            double cruiseVelocity,
            double acceleration,
            double jerk,
            double feedForwardVoltage,
            double minPose,
            double maxPose,
            double startPose,
            double tolerance,
            double statorCurrentLimit,
            double motorLimitCurrent,
            String logPath,
            double inertia,
            @SuppressWarnings("rawtypes") BaseSensor[] sensors) {

        this(
                motors,
                gear,
                positionConversionFactor,
                velocityConversionFactor,
                true,
                cruiseVelocity,
                acceleration,
                jerk,
                kP,
                kI,
                kD,
                feedForwardVoltage,
                minPose,
                maxPose,
                startPose,
                tolerance,
                statorCurrentLimit,
                true,
                motorLimitCurrent,
                logPath,
                true,
                12,
                -12,
                false, inertia, sensors);
    }

     /**
     * Use when utilizing regular PID.
     */

    public PositionSystemConstants(
            Motor[] motors,
            double gear,
            double positionConversionFactor,
            double velocityConversionFactor,
            double kP,
            double kI,
            double kD,
            double feedForwardVoltage,
            double minPose,
            double maxPose,
            double startPose,
            double tolerance,
            double statorCurrentLimit,
            double motorLimitCurrent,
            String logPath,
            double inertia,
            @SuppressWarnings("rawtypes") BaseSensor[] sensors) {

        this(
                motors,
                gear,
                positionConversionFactor,
                velocityConversionFactor,
                false,
                0,
                0,
                0,
                kP,
                kI,
                kD,
                feedForwardVoltage,
                minPose,
                maxPose,
                startPose,
                tolerance,
                statorCurrentLimit,
                true,
                motorLimitCurrent,
                logPath,
                true,
                12,
                -12,
                false, inertia, sensors);
    }

}

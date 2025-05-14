
package com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.ConstantsClasses;

import com.ma5951.utils.RobotControl.Utils.Motor;
import com.ma5951.utils.RobotControl.Utils.Sensors.BaseSensor;

public class ArmSystemConstants {

    public Motor[] MOTORS;
    public double GEAR = 1;
    public boolean IS_MOTION_MAGIC = false;
    public double KP = 0;
    public double KI = 0;
    public double KD = 0;
    public double CRUISE_VELOCITY = 0;
    public double ACCELERATION = 0;
    public double JERK = 0;
    public double FEED_FORWARD_VOLTAGE = 0;
    public double MIN_ANGLE = 0;
    public double MAX_ANGLE = 0;
    public double START_ANGLE = 0;
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
    public double ARM_LENGTH = 0.3;
    @SuppressWarnings("rawtypes")
    public BaseSensor[] SENSORS;

    public ArmSystemConstants(
            Motor[] motors,
            double gear,
            boolean isMotionMagic,
            double kP,
            double kI,
            double kD,
            double feedForwardVoltage,
            double minAngle,
            double maxAngle,
            double startAngle,
            double tolerance,
            double statorCurrentLimit,
            boolean currentLimitEnabled,
            double motorLimitCurrent,
            String logPath,
            boolean isBrake,
            double peakForwardVoltage,
            double peakReverseVoltage,
            boolean foc,
            @SuppressWarnings("rawtypes") BaseSensor[] sensors) {
        MOTORS = motors;
        GEAR = gear;
        IS_MOTION_MAGIC = isMotionMagic;
        KP = kP;
        KI = kI;
        KD = kD;
        FEED_FORWARD_VOLTAGE = feedForwardVoltage;
        MIN_ANGLE = minAngle;
        MAX_ANGLE = maxAngle;
        START_ANGLE = startAngle;
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
    }

    public ArmSystemConstants(
            Motor[] motors,
            double gear,
            double kP,
            double kI,
            double kD,
            double cruiseVelocity,
            double acceleration,
            double jerk,
            double feedForwardVoltage,
            double minAngle,
            double maxAngle,
            double startAngle,
            double tolerance,
            double statorCurrentLimit,
            boolean currentLimitEnabled,
            double motorLimitCurrent,
            String logPath,
            boolean isBrake,
            double inertia,
            double armLength,
            @SuppressWarnings("rawtypes") BaseSensor[] sensors) {
        MOTORS = motors;
        GEAR = gear;
        IS_MOTION_MAGIC = true;
        KP = kP;
        KI = kI;
        KD = kD;
        CRUISE_VELOCITY = cruiseVelocity;
        ACCELERATION = acceleration;
        JERK = jerk;
        FEED_FORWARD_VOLTAGE = feedForwardVoltage;
        MIN_ANGLE = minAngle;
        MAX_ANGLE = maxAngle;
        START_ANGLE = startAngle;
        TOLERANCE = tolerance;
        STATOR_CURRENT_LIMIT = statorCurrentLimit;
        CURRENT_LIMIT_ENABLED = currentLimitEnabled;
        MOTOR_LIMIT_CURRENT = motorLimitCurrent;
        LOG_PATH = logPath;
        IS_BRAKE = isBrake;
        INERTIA = inertia;
        SENSORS = sensors;
        ARM_LENGTH = armLength;
    }

}

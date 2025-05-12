
package com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.ConstantsClasses;


import com.ma5951.utils.RobotControl.Utils.Motor;
import com.ma5951.utils.RobotControl.Utils.Sensors.BaseSensor;

public class ArmSystemConstants {

    public Motor[] MOTORS;
    public double GEAR = 1;
    public double STATOR_CURRENT_LIMIT = 40;
    public boolean CURRENT_LIMIT_ENABLED = true;
    public double MOTOR_LIMIT_CURRENT = 40;
    public String LOG_PATH = "/Did Not Specifiyed Log Path!";
    public boolean IS_BRAKE = true;
    public double PEAK_FORWARD_VOLTAGE = 12;
    public double PEAK_REVERSE_VOLTAGE = -12;
    public boolean FOC = false;
    public double INERTIA = 0.0004;
    @SuppressWarnings("rawtypes")
    public BaseSensor[] SENSORS;

    public ArmSystemConstants(
            Motor[] motors,
            double gear,
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
            double statorCurrentLimit,
            boolean currentLimitEnabled,
            double motorLimitCurrent,
            String logPath,
            boolean isBrake,
            double inertia,
            @SuppressWarnings("rawtypes") BaseSensor[] sensors) {
        MOTORS = motors;
        GEAR = gear;
        STATOR_CURRENT_LIMIT = statorCurrentLimit;
        CURRENT_LIMIT_ENABLED = currentLimitEnabled;
        MOTOR_LIMIT_CURRENT = motorLimitCurrent;
        LOG_PATH = logPath;
        IS_BRAKE = isBrake;
        INERTIA = inertia;
        SENSORS = sensors;
    }

}

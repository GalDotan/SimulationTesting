
package com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.ConstantsClasses;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.CANBus;
import com.ma5951.utils.RobotControl.Utils.DCmotors;
import com.ma5951.utils.RobotControl.Utils.Sensors.BaseSensor;


public class RollerSystemConstants {

    public DCmotors MOTOR = DCmotors.getKrakenX60(1);
    public double GEAR = 1;
    public double STATOR_CURRENT_LIMIT = 40;
    public boolean CURRENT_LIMIT_ENABLED = true;
    public double MOTOR_LIMIT_CURRENT = 40;
    public String LOG_PATH = "/Subsystem/Did Not Specifiyed Log Path!";
    public boolean IS_REVERD = false;
    public boolean IS_BRAKE = true;
    public double PEAK_FORWARD_VOLTAGE = 12;
    public double PEAK_REVERSE_VOLTAGE = -12;
    public boolean FOC = false;
    public int MOTOR_ID = 0;
    public CANBus CAN_BUS = new CANBus("rio");
    public double INERTIA = 0.001;
    public List<BaseSensor> SENSORS = new ArrayList<>();

    public RollerSystemConstants(
            DCmotors motor,
            double gear,
            double statorCurrentLimit,
            boolean currentLimitEnabled,
            double motorLimitCurrent,
            String logPath,
            boolean isReverd,
            boolean isBrake,
            double peakForwardVoltage,
            double peakReverseVoltage,
            boolean foc,
            int motorID,
            CANBus canBus,
            List<BaseSensor> sensors) {
        MOTOR = motor;
        GEAR = gear;
        STATOR_CURRENT_LIMIT = statorCurrentLimit;
        CURRENT_LIMIT_ENABLED = currentLimitEnabled;
        MOTOR_LIMIT_CURRENT = motorLimitCurrent;
        LOG_PATH = logPath;
        IS_REVERD = isReverd;
        IS_BRAKE = isBrake;
        PEAK_FORWARD_VOLTAGE = peakForwardVoltage;
        PEAK_REVERSE_VOLTAGE = peakReverseVoltage;
        FOC = foc;
        MOTOR_ID = motorID;
        CAN_BUS = canBus;
        SENSORS = sensors;
    }

    public RollerSystemConstants(
            DCmotors motor,
            double gear,
            double statorCurrentLimit,
            boolean currentLimitEnabled,
            double motorLimitCurrent,
            String logPath,
            boolean isReverd,
            boolean isBrake,
            int motorID,
            CANBus canBus,
            List<BaseSensor> sensors) {
        MOTOR = motor;
        GEAR = gear;
        STATOR_CURRENT_LIMIT = statorCurrentLimit;
        CURRENT_LIMIT_ENABLED = currentLimitEnabled;
        MOTOR_LIMIT_CURRENT = motorLimitCurrent;
        LOG_PATH = logPath;
        IS_REVERD = isReverd;
        IS_BRAKE = isBrake;
        MOTOR_ID = motorID;
        CAN_BUS = canBus;
        SENSORS = sensors;
    }

}

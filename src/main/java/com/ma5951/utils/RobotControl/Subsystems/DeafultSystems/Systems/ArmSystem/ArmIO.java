
package com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.Systems.ArmSystem;

import com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.ConstantsClasses.ArmSystemConstants;
import com.ma5951.utils.RobotControl.Utils.Sensors.BaseSensor;

public abstract class ArmIO {

    protected ArmSystemConstants systemConstants;
    
    public ArmIO(ArmSystemConstants systemConstants) {
        this.systemConstants = systemConstants;
    }

    protected abstract void configMotors();

    public abstract double getCurrent();

    public abstract double getAppliedVolts();

    public abstract double getVelocity();

    public abstract double getPosition();

    public abstract double getSetPoint();

    public abstract void setPosition(double position);

    public abstract void setNeutralMode(boolean isBrake);

    public abstract void setVoltage(double volt);

    public abstract void setAngle(double angle);

    public abstract void setAngle(double angle, double feedForwardVoltage);

    public abstract void updatePeriodic();

    public abstract double getSensor(@SuppressWarnings("rawtypes") BaseSensor sensor);

}

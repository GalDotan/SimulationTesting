
package com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.Systems.RollerSystem;

import com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.ConstantsClasses.RollerSystemConstants;
import com.ma5951.utils.RobotControl.Utils.Sensors.BaseSensor;

public abstract class RollerIO {

    protected RollerSystemConstants systemConstants;
    
    public RollerIO(RollerSystemConstants systemConstants) {
        this.systemConstants = systemConstants;
    }

    protected abstract void configMotors();

    public abstract double getCurrent();

    public abstract double getAppliedVolts();

    public abstract double getVelocity();

    public abstract double getPosition();

    public abstract void setNeutralMode(boolean isBrake);

    public abstract void setVoltage(double volt);

    public abstract void updatePeriodic();

    public abstract double getSensor(@SuppressWarnings("rawtypes") BaseSensor sensor);

}

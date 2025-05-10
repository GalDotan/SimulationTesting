
package com.ma5951.utils.RobotControl.Utils.Sensors;

import com.ctre.phoenix6.hardware.TalonFX;

public abstract class BaseSensor {

    public void init(TalonFX motor) {
        // Do nothing
    }

    public abstract boolean get();

    public abstract void set(boolean value);
    
}

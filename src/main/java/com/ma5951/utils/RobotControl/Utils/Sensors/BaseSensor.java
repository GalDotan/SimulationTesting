
package com.ma5951.utils.RobotControl.Utils.Sensors;


public abstract class BaseSensor {

    public abstract java.io.Serializable get();

    public abstract void set(java.io.Serializable value);

    public abstract String getType();

    public abstract String getName();
    
}

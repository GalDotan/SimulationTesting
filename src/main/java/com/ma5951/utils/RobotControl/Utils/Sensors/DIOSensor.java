
package com.ma5951.utils.RobotControl.Utils.Sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Robot;

public class DIOSensor extends BaseSensor {

    private DigitalInput dioSensor;
    private boolean value;
    private boolean invert;
    private String name;

    public DIOSensor(String name,int id , boolean invert) {
        dioSensor = new DigitalInput(id);
        this.invert = invert;
        this.name = name;

    }

    @Override
    public java.io.Serializable get() {
        if (Robot.isSimulation()) {
            return value;
        }
        return invert ? !dioSensor.get() : dioSensor.get();
    }

    @Override
    public void set(java.io.Serializable value) {
        this.value = (Boolean)value;
    }

    public String getType() {
        return "DIOSensor";
    }
    
    @Override
    public String getName() {
        return name;
    }

}
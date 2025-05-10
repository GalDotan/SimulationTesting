
package com.ma5951.utils.RobotControl.Utils.Sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Robot;

public class DIOSensor extends BaseSensor {

    private DigitalInput dioSensor;
    private boolean value;
    private boolean invert;

    public DIOSensor(int id , boolean invert) {
        dioSensor = new DigitalInput(id);
        this.invert = invert;
    }

    @Override
    public boolean get() {
        if (Robot.isSimulation()) {
            return value;
        }
        return invert ? !dioSensor.get() : dioSensor.get();
    }

    @Override
    public void set(boolean value) {
        this.value = value;
    }

}
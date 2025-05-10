
package com.ma5951.utils.RobotControl.Utils.Sensors;


import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ma5951.utils.RobotControl.Utils.StatusSignalsRunner;

import frc.robot.Robot;

public class TalonLimitSensor extends BaseSensor{

    public enum LimitSwitchSource {
        FORWARD,
        REVERSE
    }

    private LimitSwitchSource limitSwitchSource;
    private boolean value;
    @SuppressWarnings("rawtypes")
    private StatusSignal signal;
    private Object equals;
    private boolean invert;

    public TalonLimitSensor(LimitSwitchSource limitSwitchSource, boolean invert) {
        this.limitSwitchSource = limitSwitchSource;
        this.invert = invert;
    }

    @Override
    public void init(TalonFX motor) {
        if (LimitSwitchSource.FORWARD == limitSwitchSource) {
            signal = motor.getForwardLimit(false);
            equals = invert ? ForwardLimitValue.Open : ForwardLimitValue.ClosedToGround;
        } else if (LimitSwitchSource.REVERSE == limitSwitchSource) {
            signal = motor.getReverseLimit(false);
            equals = invert ? ForwardLimitValue.Open : ForwardLimitValue.ClosedToGround;
        }

        StatusSignalsRunner.registerSignals(signal);
    }

    @Override
    public boolean get() {
        if (Robot.isSimulation()) {
            return value;
        }
        return signal.getValue().equals(equals);
    }

    @Override
    public void set(boolean value) {
        this.value = value;
    }


}

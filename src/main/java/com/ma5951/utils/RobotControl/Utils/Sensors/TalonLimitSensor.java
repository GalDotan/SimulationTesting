
package com.ma5951.utils.RobotControl.Utils.Sensors;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.ma5951.utils.RobotControl.Utils.Motor;
import com.ma5951.utils.RobotControl.Utils.StatusSignalsRunner;

import frc.robot.Robot;

public class TalonLimitSensor extends BaseSensor {

    public enum LimitSwitchSource {
        FORWARD,
        REVERSE
    }

    private boolean value;
    @SuppressWarnings("rawtypes")
    private StatusSignal signal;
    private Object equals;
    private String name;
    private HardwareLimitSwitchConfigs configuration;

    public TalonLimitSensor(String name,LimitSwitchSource limitSwitchSource, boolean invert, Motor motor) {
        this.name = name;
        configuration.ForwardLimitEnable = false;
        configuration.ReverseLimitEnable = false;
        motor.talonFX.getConfigurator().apply(configuration);

        if (LimitSwitchSource.FORWARD == limitSwitchSource) {
            signal = motor.talonFX.getForwardLimit(false);
            equals = invert ? ForwardLimitValue.Open : ForwardLimitValue.ClosedToGround;
        } else if (LimitSwitchSource.REVERSE == limitSwitchSource) {
            signal = motor.talonFX.getReverseLimit(false);
            equals = invert ? ReverseLimitValue.Open : ReverseLimitValue.ClosedToGround;
        }

        StatusSignalsRunner.registerSignals(signal);
    }

    @Override
    public java.io.Serializable get() {
        if (Robot.isSimulation()) {
            return value;
        }
        return signal.getValue().equals(equals);
    }

    @Override
    public void set(java.io.Serializable value) {
        this.value = (Boolean)value;
    }

    @Override
    public String getType() {
        return "TalonLimitSensor";
    }

    @Override
    public String getName() {
        return name;
    }

}

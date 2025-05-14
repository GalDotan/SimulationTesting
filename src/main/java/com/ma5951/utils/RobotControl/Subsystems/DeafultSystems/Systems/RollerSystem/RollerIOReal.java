package com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.Systems.RollerSystem;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ma5951.utils.Logger.MALog;
import com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.ConstantsClasses.RollerSystemConstants;
import com.ma5951.utils.RobotControl.Utils.Motor;
import com.ma5951.utils.RobotControl.Utils.StatusSignalsRunner;
import com.ma5951.utils.RobotControl.Utils.Sensors.BaseSensor;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class RollerIOReal extends RollerIO {

    private final int numOfMotors;
    private final VoltageOut voltageRequest = new VoltageOut(0);
    protected final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    private final StrictFollower[] followers;

    private StatusSignal<AngularVelocity> motorVelocity;
    private StatusSignal<Angle> motorPosition;
    private StatusSignal<Current> motorCurrent;
    private StatusSignal<Voltage> motorVoltage;

    private int i = 0;

    public RollerIOReal(RollerSystemConstants systemConstants) {
        super(systemConstants);
        numOfMotors = systemConstants.MOTORS.length;

        configMotors();

        followers = new StrictFollower[numOfMotors - 1];

        for (Motor motor : systemConstants.MOTORS) {
            if (i > 0) {
                followers[i - 1] = new StrictFollower(systemConstants.MOTORS[0].talonFX.getDeviceID());
            } else {
                motorVelocity = motor.talonFX.getVelocity(false);
                motorCurrent = motor.talonFX.getStatorCurrent(false);
                motorVoltage = motor.talonFX.getMotorVoltage(false);
                motorPosition = motor.talonFX.getPosition(false);
                StatusSignalsRunner.registerSignals(motorVelocity, motorCurrent,
                        motorVoltage);
            }
            TalonFX.resetSignalFrequenciesForAll(motor.talonFX);
            motorConfig.MotorOutput.Inverted = motor.direction;
            motor.talonFX.getConfigurator().apply(motorConfig);
            i++;
        }

    }

    @Override
    protected void configMotors() {
        motorConfig.Feedback.SensorToMechanismRatio = systemConstants.GEAR;

        motorConfig.MotorOutput.NeutralMode = systemConstants.IS_BRAKE
                ? NeutralModeValue.Brake
                : NeutralModeValue.Coast;

        motorConfig.Voltage.PeakForwardVoltage = systemConstants.PEAK_FORWARD_VOLTAGE;
        motorConfig.Voltage.PeakReverseVoltage = systemConstants.PEAK_REVERSE_VOLTAGE;

        motorConfig.CurrentLimits.StatorCurrentLimit = systemConstants.STATOR_CURRENT_LIMIT;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = systemConstants.CURRENT_LIMIT_ENABLED;
    }

    @Override
    public double getSensor(@SuppressWarnings("rawtypes") BaseSensor sensor) {
        return sensor.get();
    }

    @Override
    public double getCurrent() {
        return motorCurrent.getValueAsDouble();
    }

    @Override
    public double getAppliedVolts() {
        return motorVoltage.getValueAsDouble();
    }

    @Override
    public double getVelocity() {
        return motorVelocity.getValueAsDouble() * 60;
    }

    @Override
    public double getPosition() {
        return motorPosition.getValueAsDouble() * 360; 
    }

    @Override
    public void setNeutralMode(boolean isBrake) {
        motorConfig.MotorOutput.NeutralMode = isBrake
                ? NeutralModeValue.Brake
                : NeutralModeValue.Coast;
        for (Motor motor : systemConstants.MOTORS) {
            motorConfig.MotorOutput.Inverted = motor.direction;
            motor.talonFX.getConfigurator().apply(motorConfig);
        }
    }

    @Override
    public void setVoltage(double volt) {
        systemConstants.MOTORS[0].talonFX.setControl(voltageRequest.withOutput(volt)
                .withLimitForwardMotion(getCurrent() > systemConstants.MOTOR_LIMIT_CURRENT)
                .withLimitReverseMotion(getCurrent() > systemConstants.MOTOR_LIMIT_CURRENT));
        i = 1;
        while (i < numOfMotors) {
            systemConstants.MOTORS[i].talonFX.setControl(followers[i - 1]);
        }
    }

    @Override
    public void updatePeriodic() {
        MALog.log("/Subsystem/" + systemConstants.LOG_PATH + "/IO/" + "/Velocity", getVelocity());
        MALog.log("/Subsystem/" + systemConstants.LOG_PATH + "/IO/" + "/Voltage", getAppliedVolts());
        MALog.log("/Subsystem/" + systemConstants.LOG_PATH + "/IO/" + "/Current", getCurrent());
        MALog.log("/Subsystem/" + systemConstants.LOG_PATH + "/IO/" + "/Position", getPosition());

        for (@SuppressWarnings("rawtypes")
        BaseSensor sensor : systemConstants.SENSORS) {
            if (sensor.getType().equals("BooleanSensor")) {
                MALog.log("/Subsystem/" + systemConstants.LOG_PATH + "/IO/" + sensor.getName(),
                        (sensor.get() == 1 ? true : false));
            } else {
                MALog.log("/Subsystem/" + systemConstants.LOG_PATH + "/IO/" + sensor.getName(), sensor.get());
            }
        }

    }

}

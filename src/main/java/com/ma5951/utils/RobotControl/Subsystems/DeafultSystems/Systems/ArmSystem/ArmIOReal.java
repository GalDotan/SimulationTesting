package com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.Systems.ArmSystem;

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
import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ArmIOReal extends ArmIO {

    private final int numOfMotors;
    private final VoltageOut voltageRequest = new VoltageOut(0);
    protected final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    private final StrictFollower[] followers;

    private final StatusSignal<AngularVelocity>[] motorVelocity;
    private final StatusSignal<Current>[] motorCurrent;
    private final StatusSignal<Voltage>[] motorVoltage;

    private double velocitySum;
    private double currentSum;
    private double voltageSum;

    private int i = 0;

    @SuppressWarnings("unchecked")
    public ArmIOReal(RollerSystemConstants systemConstants) {
        super(systemConstants);
        numOfMotors = systemConstants.MOTORS.length;

        configMotors();

        
        followers = new StrictFollower[numOfMotors - 1];
        motorVelocity = new StatusSignal[numOfMotors];
        motorCurrent = new StatusSignal[numOfMotors];
        motorVoltage = new StatusSignal[numOfMotors];


        for (Motor motor : systemConstants.MOTORS) {
            motorVelocity[i] = motor.talonFX.getVelocity(false);
            motorCurrent[i] = motor.talonFX.getStatorCurrent(false);
            motorVoltage[i] = motor.talonFX.getMotorVoltage(false);
            TalonFX.resetSignalFrequenciesForAll(motor.talonFX);
            motorConfig.MotorOutput.Inverted = motor.direction;
            motor.talonFX.getConfigurator().apply(motorConfig);
            StatusSignalsRunner.registerSignals(motorVelocity[i] , motorCurrent[i] ,
            motorVoltage[i]);
            if (i > 0) {
                followers[i - 1] = new StrictFollower(systemConstants.MOTORS[0].talonFX.getDeviceID());
            }
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
        currentSum = 0;
        for (StatusSignal<Current> motorCurrent : motorCurrent) {
            currentSum += motorCurrent.getValueAsDouble();
        }
        return currentSum / numOfMotors;
    }

    @Override
    public double getAppliedVolts() {
        voltageSum = 0;
        for (StatusSignal<Voltage> motorVoltag : motorVoltage) {
            voltageSum += motorVoltag.getValueAsDouble();
        }
        return voltageSum / numOfMotors;
    }

    @Override
    public double getVelocity() {
        velocitySum = 0;
        for (StatusSignal<AngularVelocity> motorVelo : motorVelocity) {
            velocitySum += motorVelo.getValueAsDouble();
        }
        return ConvUtil.RPStoRPM(velocitySum / numOfMotors);
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

        for (@SuppressWarnings("rawtypes") BaseSensor sensor : systemConstants.SENSORS) {
            if (sensor.getType().equals("BooleanSensor")) {
                MALog.log("/Subsystem/" + systemConstants.LOG_PATH + "/IO/" + sensor.getName(), (sensor.get() == 1 ? true : false));
            } else {
                MALog.log("/Subsystem/" + systemConstants.LOG_PATH + "/IO/" + sensor.getName(), sensor.get());
            }
        }

    }

}

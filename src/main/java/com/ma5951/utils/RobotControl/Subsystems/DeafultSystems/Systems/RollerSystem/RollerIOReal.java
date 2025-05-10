package com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.Systems.RollerSystem;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ma5951.utils.Logger.MALog;
import com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.ConstantsClasses.RollerSystemConstants;
import com.ma5951.utils.RobotControl.Utils.StatusSignalsRunner;
import com.ma5951.utils.RobotControl.Utils.Sensors.BaseSensor;
import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class RollerIOReal extends RollerIO {

    private final TalonFX motor;
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Current> motorCurrent;
    private final StatusSignal<Voltage> motorVoltage;
    private final List<BaseSensor> sensors = new ArrayList<>();

    public RollerIOReal(RollerSystemConstants systemConstants) {
        super(systemConstants);

        this.motor = new TalonFX(systemConstants.MOTOR_ID, systemConstants.CAN_BUS);

        this.motorVelocity = motor.getVelocity();
        this.motorCurrent = motor.getStatorCurrent();
        this.motorVoltage = motor.getMotorVoltage();

        TalonFX.resetSignalFrequenciesForAll(motor);
        configMotors();
        TalonFX.optimizeBusUtilizationForAll(motor);
    }

    @Override
    protected void configMotors() {
        motorConfig.Feedback.SensorToMechanismRatio = systemConstants.GEAR;

        motorConfig.MotorOutput.Inverted = systemConstants.IS_REVERD
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        motorConfig.MotorOutput.NeutralMode = systemConstants.IS_BRAKE
                ? NeutralModeValue.Brake
                : NeutralModeValue.Coast;

        motorConfig.Voltage.PeakForwardVoltage = systemConstants.PEAK_FORWARD_VOLTAGE;
        motorConfig.Voltage.PeakReverseVoltage = systemConstants.PEAK_REVERSE_VOLTAGE;

        if (systemConstants.MOTOR.motor == DCMotor.getFalcon500(1) || systemConstants.MOTOR.motor == DCMotor.getFalcon500(1))

        motorConfig.CurrentLimits.StatorCurrentLimit = systemConstants.STATOR_CURRENT_LIMIT;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = systemConstants.CURRENT_LIMIT_ENABLED;

        
        motor.getConfigurator().apply(motorConfig);
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
        return ConvUtil.RPStoRPM(motorVelocity.getValueAsDouble());
    }

    @Override
    public void setNeutralMode(boolean isBrake) {
        motorConfig.MotorOutput.NeutralMode = isBrake
                ? NeutralModeValue.Brake
                : NeutralModeValue.Coast;
        motor.getConfigurator().apply(motorConfig);
    }

    @Override
    public void setVoltage(double volt) {
        motor.setControl(voltageRequest.withOutput(volt));
    }

    @Override
    public void updatePeriodic() {
        StatusSignalsRunner.registerSignals(
            motorCurrent,
            motorVoltage,
            motorVelocity
        );

        MALog.log(systemConstants.LOG_PATH + "/Roller Velocity", getVelocity());
        MALog.log(systemConstants.LOG_PATH + "/Roller Voltage", getAppliedVolts());
        MALog.log(systemConstants.LOG_PATH + "/Roller Current", getCurrent());
    }
}

package frc.robot.Subsystem.Gripper.IOs;

import com.ctre.phoenix6.hardware.TalonFX;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ma5951.utils.Logger.MALog;
import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.PortMap;
import frc.robot.Subsystem.Gripper.GripperConstants;

public class GripperIOReal implements GripperIO {


    protected TalonFX rollerMotor;
    protected TalonFXConfiguration rollerConfig;

    private DigitalInput coralSensor;


    private StatusSignal<AngularVelocity> rollerMotorVelocity;
    private StatusSignal<Current> rollerMotorCurrent;
    private StatusSignal<Voltage> rollerMotorVoltage;


    public GripperIOReal() {

        rollerMotor = new TalonFX(PortMap.Gripper.GripperMotor, PortMap.CanBus.RioBus);
        rollerConfig = new TalonFXConfiguration();


        coralSensor = new DigitalInput(PortMap.Gripper.CoralSensor);

        rollerMotorVelocity = rollerMotor.getVelocity();
        rollerMotorCurrent = rollerMotor.getStatorCurrent();
        rollerMotorVoltage = rollerMotor.getMotorVoltage();



        configRollerMotor();
    }

    private void configRollerMotor() {
        rollerConfig.Feedback.SensorToMechanismRatio = GripperConstants.GEAR_RATIO;

        rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;


        rollerConfig.CurrentLimits.StatorCurrentLimit = GripperConstants.STATOR_CURRENT_LIMIT;
        rollerConfig.CurrentLimits.StatorCurrentLimitEnable = GripperConstants.ENABLE_CURRENT_LIMIT;


        rollerMotor.getConfigurator().apply(rollerConfig);
    }


    public boolean hasCoral() {
        return coralSensor.get();
    }


    public double getVelocity() {
        return ConvUtil.RPStoRPM(rollerMotorVelocity.getValueAsDouble());
    }

    public double getCurrent() {
        return rollerMotorCurrent.getValueAsDouble();
    }

    public double getAppliedVolts() {
        return rollerMotorVoltage.getValueAsDouble();
    }

    public void setNeutralMode(boolean isBrake) {
        rollerConfig.MotorOutput.NeutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        rollerMotor.getConfigurator().apply(rollerConfig);
    }

    public void setVoltage(double volt) {
        rollerMotor.setVoltage(volt);
        ;
    }

    public void updatePeriodic() {
        BaseStatusSignal.refreshAll(
                rollerMotorCurrent,
                rollerMotorVelocity,
                rollerMotorVoltage);


        MALog.log("Subsystems/Gripper/IO/Gripper Velocity", getVelocity());
        MALog.log("Subsystems/Gripper/IO/Gripper Voltage", getAppliedVolts());
        MALog.log("Subsystems/Gripper/IO/Gripper Current", getCurrent());
        MALog.log("Subsystems/Gripper/IO/Has Coral", hasCoral());

    }
}

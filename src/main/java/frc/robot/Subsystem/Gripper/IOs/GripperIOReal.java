package frc.robot.Subsystem.Gripper.IOs;

import com.ctre.phoenix6.hardware.TalonFX;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ma5951.utils.Logger.MALog;
import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.PortMap;
import frc.robot.Subsystem.Intake.IntakeArm.IntakeArmConstants;
import frc.robot.Subsystem.Intake.IntakeRoller.IntakeRollerConstants;

public class GripperIOReal implements GripperIO {


    protected TalonFX rollerMotor;
    protected TalonFXConfiguration rollerConfig;

    private DigitalInput coralSensor;

    private VoltageOut voltageRequest;

    private StatusSignal<AngularVelocity> rollerMotorVelocity;
    private StatusSignal<Current> rollerMotorCurrent;
    private StatusSignal<Voltage> rollerMotorVoltage;


    public GripperIOReal() {

        rollerMotor = new TalonFX(PortMap.Intake.RollerMotor, PortMap.CanBus.RioBus);
        rollerConfig = new TalonFXConfiguration();

        voltageRequest = new VoltageOut(0);

        coralSensor = new DigitalInput(PortMap.Gripper.CoralSensor);

        rollerMotorVelocity = rollerMotor.getVelocity();
        rollerMotorCurrent = rollerMotor.getStatorCurrent();
        rollerMotorVoltage = rollerMotor.getMotorVoltage();



        configRollerMotor();
    }

    private void configRollerMotor() {
        rollerConfig.Feedback.SensorToMechanismRatio = IntakeArmConstants.ROLLER_GEAR_RATIO;

        rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;


        rollerConfig.CurrentLimits.StatorCurrentLimit = IntakeArmConstants.ROLLER_STATOR_CURRENT_LIMIT;
        rollerConfig.CurrentLimits.StatorCurrentLimitEnable = IntakeArmConstants.ROLLER_ENABLE_CURRENT_LIMIT;


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
        rollerMotor.setControl(voltageRequest.withOutput(volt)
        .withLimitForwardMotion(getCurrent() > IntakeRollerConstants.k_CAN_MOVE_CURRENT_LIMIT)
        .withLimitReverseMotion(getCurrent() < IntakeRollerConstants.k_CAN_MOVE_CURRENT_LIMIT));
    }

    public void updatePeriodic() {
        BaseStatusSignal.refreshAll(
                rollerMotorCurrent,
                rollerMotorVelocity,
                rollerMotorVoltage);


        MALog.log("/Subsystems/Gripper/IO/Roller Velocity", getVelocity());
        MALog.log("Subsystems/Gripper/IO/Roller Voltage", getAppliedVolts());
        MALog.log("Subsystems/Gripper/IO/Roller Current", getCurrent());

    }
}

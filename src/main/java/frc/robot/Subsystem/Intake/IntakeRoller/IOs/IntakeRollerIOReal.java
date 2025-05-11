package frc.robot.Subsystem.Intake.IntakeRoller.IOs;

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
import frc.robot.Subsystem.Intake.IntakeRoller.IntakeRollerConstants;

public class IntakeRollerIOReal implements IntakeRollerIO {


    protected TalonFX rollerMotor;
    protected TalonFXConfiguration rollerConfig;

    private DigitalInput coralSensor;



    private StatusSignal<AngularVelocity> rollerMotorVelocity;
    private StatusSignal<Current> rollerMotorCurrent;
    private StatusSignal<Voltage> rollerMotorVoltage;


    public IntakeRollerIOReal() {

        rollerMotor = new TalonFX(PortMap.Intake.RollerMotor, PortMap.CanBus.RioBus);
        rollerConfig = new TalonFXConfiguration();

        coralSensor = new DigitalInput(PortMap.Intake.CoralSensor);

        rollerMotorVelocity = rollerMotor.getVelocity();
        rollerMotorCurrent = rollerMotor.getStatorCurrent();
        rollerMotorVoltage = rollerMotor.getMotorVoltage();



        configRollerMotor();
    }

    private void configRollerMotor() {
        rollerConfig.Feedback.SensorToMechanismRatio = IntakeRollerConstants.GEAR_RATIO;

        rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;


        rollerConfig.CurrentLimits.StatorCurrentLimit = IntakeRollerConstants.STATOR_CURRENT_LIMIT;
        rollerConfig.CurrentLimits.StatorCurrentLimitEnable = IntakeRollerConstants.ENABLE_CURRENT_LIMIT;


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
        
    }

    public void updatePeriodic() {
        BaseStatusSignal.refreshAll(
                rollerMotorCurrent,
                rollerMotorVelocity,
                rollerMotorVoltage);


        MALog.log("Subsystems/Intake/Intake Roller/IO/Roller Velocity", getVelocity());
        MALog.log("Subsystems/Intake/Intake Roller/IO/Roller Voltage", getAppliedVolts());
        MALog.log("Subsystems/Intake/Intake Roller/IO/Roller Current", getCurrent());
        MALog.log("Subsystems/Intake/Intake Roller/IO/Has Coral", hasCoral());

    }
}

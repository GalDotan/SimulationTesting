package frc.robot.Subsystem.Intake.IntakeArm.IOs;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ma5951.utils.Logger.MALog;
import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.PortMap;
import frc.robot.Subsystem.Intake.IntakeArm.IntakeArmConstants;

public class IntakeArmIOReal implements IntakeArmIO {

    protected TalonFX armMotor;
    protected TalonFXConfiguration armConfig;

    private PositionVoltage positionControl;

    protected CANcoder absEncoder;

    private StatusSignal<Angle> armMotorPosition;
    private StatusSignal<Current> armMotorCurrent;
    private StatusSignal<Voltage> armMotorVoltage;
    private StatusSignal<Double> setPoint;
    private StatusSignal<Angle> encoderPosition;


    public IntakeArmIOReal() {
        armMotor = new TalonFX(PortMap.Intake.ArmMotor, PortMap.CanBus.RioBus);
        armConfig = new TalonFXConfiguration();
        absEncoder = new CANcoder(PortMap.Intake.ArmCanCoder, PortMap.CanBus.RioBus);
        positionControl = new PositionVoltage(0); 


        armMotorPosition = armMotor.getPosition();
        armMotorCurrent = armMotor.getStatorCurrent();
        armMotorVoltage = armMotor.getMotorVoltage();
        setPoint = armMotor.getClosedLoopReference();
        encoderPosition = absEncoder.getAbsolutePosition();


        encoderPosition.refresh();
        armMotor.setPosition(0);

        configArmMotor();
    }

    private void configArmMotor() {
        armConfig.Feedback.SensorToMechanismRatio = IntakeArmConstants.ARM_GEAR_RATIO;


        armConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;


        armConfig.Slot0.kP = IntakeArmConstants.kP;
        armConfig.Slot0.kI = IntakeArmConstants.kI;
        armConfig.Slot0.kD = IntakeArmConstants.kD;

        armConfig.CurrentLimits.StatorCurrentLimit = IntakeArmConstants.ARM_STATOR_CURRENT_LIMIT;
        armConfig.CurrentLimits.StatorCurrentLimitEnable = IntakeArmConstants.ARM_ENABLE_CURRENT_LIMIT;


        armMotor.getConfigurator().apply(armConfig);
    }

    public double getAbsolutePosition() {
            return -ConvUtil.RotationsToDegrees(encoderPosition.getValueAsDouble()); 
       
    }

    public double getCurrent() {
        return armMotorCurrent.getValueAsDouble();
    }

    public double getPosition() {
        return ConvUtil.RotationsToDegrees(armMotorPosition.getValueAsDouble());
    }

    public double getAppliedVolts() {
        return armMotorVoltage.getValueAsDouble();
    }

    public double getError() {
        return getSetPoint() - getPosition();
    }

    public double getSetPoint() {
        return ConvUtil.RotationsToDegrees(setPoint.getValueAsDouble());
    }

    public void resetPosition(double newAngle) {
        armMotor.setPosition(ConvUtil.DegreesToRotations(newAngle));
    }

    public void updatePID(double Kp, double Ki, double Kd) {
        armConfig.Slot0.kP = Kp;
        armConfig.Slot0.kI = Ki;
        armConfig.Slot0.kD = Kd;
        armMotor.getConfigurator().apply(armConfig);
    }

    public void setNeutralMode(boolean isBrake) {
        armConfig.MotorOutput.NeutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        armMotor.getConfigurator().apply(armConfig);
    }

    public void setVoltage(double volt) {
        armMotor.setVoltage(volt);
    }

    public void setAngle(double angle , double FEED_FORWARD) {
        armMotor.setControl(
                positionControl.withPosition(ConvUtil.DegreesToRotations(angle)).withSlot(IntakeArmConstants.CONTROL_SLOT)
                .withFeedForward(IntakeArmConstants.FEED_FORWARD_VOLTAGE)
                .withLimitForwardMotion(getPosition() > IntakeArmConstants.MAX_ANGLE || Math.abs(getCurrent()) > IntakeArmConstants.k_CAN_MOVE_CURRENT_LIMIT)
                .withLimitReverseMotion(getPosition() < IntakeArmConstants.MIN_ANGLE || Math.abs(getCurrent()) > IntakeArmConstants.k_CAN_MOVE_CURRENT_LIMIT));
    }

    public void updatePeriodic() {
        BaseStatusSignal.refreshAll(
                armMotorCurrent,
                armMotorPosition,
                armMotorVoltage,
                setPoint,
                encoderPosition);

        MALog.log("Subsystems/Intake/Intake Arm/IO/Arm Position", getPosition());
        MALog.log("Subsystems/Intake/Intake Arm/IO/Arm Voltage", getAppliedVolts());
        MALog.log("Subsystems/Intake/Intake Arm/IO/Arm Current", getCurrent());
        MALog.log("Subsystems/Intake/Intake Arm/IO/Arm Set Point", getSetPoint());
        MALog.log("Subsystems/Intake/Intake Arm/IO/Arm Error", getError());

    }
}

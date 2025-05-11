
package frc.robot.Subsystem.Elevator.IOs;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ma5951.utils.Logger.MALog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.PortMap;
import frc.robot.Subsystem.Elevator.ElevatorConstants;

public class ElevatorIOReal implements ElevatorIO {

    protected TalonFX masterMotor;

    private PositionVoltage positionVoltage; 
    protected TalonFXConfiguration masterConfig;

    private StatusSignal<Current> masterMotorCurrent;
    private StatusSignal<Angle> masterMotorPosition;
    private StatusSignal<AngularVelocity> masterMotorVelocity;
    private StatusSignal<Voltage> masterMotorAppliedVoltage;
    private StatusSignal<Double> masterError;
    private StatusSignal<Double> masterSetPoint;

    public ElevatorIOReal() {
        masterMotor = new TalonFX(PortMap.Elevator.ElevatorMotor, PortMap.CanBus.CANivoreBus);
        masterConfig = new TalonFXConfiguration();
        positionVoltage = new PositionVoltage(0);

        masterMotorCurrent = masterMotor.getStatorCurrent();
        masterMotorPosition = masterMotor.getPosition();
        masterMotorVelocity = masterMotor.getVelocity();
        masterMotorAppliedVoltage = masterMotor.getMotorVoltage();
        masterError = masterMotor.getClosedLoopError();
        masterSetPoint = masterMotor.getClosedLoopReference();


        configMotors();

        masterMotor.setPosition(0);
    }

    public void configMotors() {
        masterConfig.Feedback.SensorToMechanismRatio = ElevatorConstants.GEAR * ElevatorConstants.SPROKET_CIRCUMFERENCE;

        masterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        masterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        masterConfig.Slot0.kP = ElevatorConstants.kP;
        masterConfig.Slot0.kI = ElevatorConstants.kI;
        masterConfig.Slot0.kD = ElevatorConstants.kD;
        masterConfig.Slot0.kS = ElevatorConstants.kS;
        
        masterConfig.CurrentLimits.SupplyCurrentLimitEnable = ElevatorConstants.ENABLE_CURRENT_LIMIT;
        masterConfig.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.CURRENT_LIMIT;
        masterConfig.CurrentLimits.SupplyCurrentLowerLimit = ElevatorConstants.CONTINUOUS_LOWER_LIMIT;
        masterConfig.CurrentLimits.SupplyCurrentLowerTime = ElevatorConstants.CONTINUOUS_CURRENT_DURATION;

        masterMotor.getConfigurator().apply(masterConfig);
    }

    public double getCurrent() {
        return masterMotorCurrent.getValueAsDouble();
    }

    public double getPosition() {
        return masterMotorPosition.getValueAsDouble()  * 2 / 100;
    }

    public double getVelocity() {
        return masterMotorVelocity.getValueAsDouble();
    }

    public double getAppliedVolts() {
        return masterMotorAppliedVoltage.getValueAsDouble();
    }

    public double getError() {
        return (positionVoltage.Position * 2 / 100) - getPosition(); 
    }

    
    public double getSetPoint() {
        return masterSetPoint.getValueAsDouble()* 2 / 100; 
    }

    public void resetPosition(double newHight) {
        masterMotor.setPosition(newHight);
    }

    public void setVoltage(double volt) {
        masterMotor.setVoltage(volt);
    }

    public void updatePID(double Kp , double Ki , double Kd) {
        masterConfig.Slot0.kP = Kp;
        masterConfig.Slot0.kI = Ki;
        masterConfig.Slot0.kD = Kd;

        masterMotor.getConfigurator().apply(masterConfig);
    }

    public void setNutralMode(boolean isbrake) {
        if (isbrake) {
            masterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        } else {
            masterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        }

        masterMotor.getConfigurator().apply(masterConfig);
    }

    public void setHight(double hight) {
        masterMotor.setControl(positionVoltage.withPosition((hight / 2 * 100 ) ).withSlot(ElevatorConstants.CONTROL_SLOT));
    }

    public void updatePeriodic() {
        BaseStatusSignal.refreshAll(
            masterMotorCurrent,
            masterMotorPosition,
            masterMotorVelocity,
            masterMotorAppliedVoltage,
            masterError,
            masterSetPoint
        );

        MALog.log("/Subsystems/Elevator/IO/Motor Current", getCurrent());
        MALog.log("/Subsystems/Elevator/IO/Motor Position", getPosition());
        MALog.log("/Subsystems/Elevator/IO/Motor Velocity", getVelocity());
        MALog.log("/Subsystems/Elevator/IO/Motor Applied Voltage", getAppliedVolts());
        MALog.log("/Subsystems/Elevator/IO/Motor Error", getError());
        MALog.log("/Subsystems/Elevator/IO/Motor Set Point", getSetPoint());


    }
}


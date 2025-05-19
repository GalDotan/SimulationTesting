
package com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.Systems.PositionControlled;

import com.ma5951.utils.RobotControl.Subsystems.StateControlledSubsystem;
import com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.ConstantsClasses.PositionSystemConstants;

import frc.robot.Robot;

public class PositionControlledSubsystem extends StateControlledSubsystem{

    protected PositionSystemConstants systemConstants;
    protected PositionControlledIO positionIO;

    public PositionControlledSubsystem(String name,PositionSystemConstants systemConstants) {
        super(name);
        this.systemConstants = systemConstants;
        positionIO = Robot.isSimulation() ? new PositionControllerSim(systemConstants) : new PositionControllerReal(systemConstants);
    }

    /**
     * Used to override the deafult SIM IO with a custom one
     */

    public PositionControlledSubsystem(String name,PositionSystemConstants systemConstants, PositionControlledIO positionIOSIM) {
        super(name);
        this.systemConstants = systemConstants;
        positionIO = Robot.isSimulation() ? positionIOSIM : new PositionControllerReal(systemConstants);
    }


    public double getCurrent() {
        return positionIO.getCurrent();
    }
    public double getAppliedVolts() {
        return positionIO.getAppliedVolts();
    }
    public double getVelocity() {
        return positionIO.getVelocity();
    }
    public void setNeutralMode(boolean isBrake) {
        positionIO.setNeutralMode(isBrake);
    }
    public void setVoltage(double volt) {
        positionIO.setVoltage(volt);
    }
    public void updatePeriodic() {
        positionIO.updatePeriodic();
    }
    public double getSensor(int sensorIndedx) {
        return positionIO.getSensor(sensorIndedx);
    }

    @Override
    public void periodic() {
        super.periodic();
        positionIO.updatePeriodic();
    }
    

}

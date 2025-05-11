
package com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.Systems.RollerSystem;

import com.ma5951.utils.RobotControl.Subsystems.StateControlledSubsystem;
import com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.ConstantsClasses.RollerSystemConstants;
import com.ma5951.utils.RobotControl.Utils.Sensors.BaseSensor;

import frc.robot.Robot;

public class RollerSubsystem extends StateControlledSubsystem{

    protected RollerSystemConstants systemConstants;
    protected RollerIO rollerIO;

    public RollerSubsystem(String name,RollerSystemConstants systemConstants) {
        super(name);
        this.systemConstants = systemConstants;
        rollerIO = Robot.isSimulation() ? new RollerIOSim(systemConstants) : new RollerIOReal(systemConstants);
    }

    public double getCurrent() {
        return rollerIO.getCurrent();
    }
    public double getAppliedVolts() {
        return rollerIO.getAppliedVolts();
    }
    public double getVelocity() {
        return rollerIO.getVelocity();
    }
    public void setNeutralMode(boolean isBrake) {
        rollerIO.setNeutralMode(isBrake);
    }
    public void setVoltage(double volt) {
        rollerIO.setVoltage(volt);
    }
    public void updatePeriodic() {
        rollerIO.updatePeriodic();
    }
    public double getSensor(BaseSensor sensor) {
        return rollerIO.getSensor(sensor);
    }
    public RollerIO getRollerIO() {
        return rollerIO;
    }

    @Override
    public void periodic() {
        super.periodic();
        rollerIO.updatePeriodic();
    }
    

}

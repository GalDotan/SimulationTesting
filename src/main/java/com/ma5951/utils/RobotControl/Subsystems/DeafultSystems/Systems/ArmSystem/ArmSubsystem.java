
package com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.Systems.ArmSystem;

import com.ma5951.utils.RobotControl.Subsystems.StateControlledSubsystem;
import com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.ConstantsClasses.RollerSystemConstants;
import com.ma5951.utils.RobotControl.Utils.Sensors.BaseSensor;

import frc.robot.Robot;

public class ArmSubsystem extends StateControlledSubsystem{

    protected RollerSystemConstants systemConstants;
    protected ArmIO rollerIO;

    public ArmSubsystem(String name,RollerSystemConstants systemConstants) {
        super(name);
        this.systemConstants = systemConstants;
        rollerIO = Robot.isSimulation() ? new ArmIOSim(systemConstants) : new ArmIOReal(systemConstants);
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
    public double getSensor(@SuppressWarnings("rawtypes") BaseSensor sensor) {
        return rollerIO.getSensor(sensor);
    }
    public ArmIO getRollerIO() {
        return rollerIO;
    }

    @Override
    public void periodic() {
        super.periodic();
        rollerIO.updatePeriodic();
    }
    

}

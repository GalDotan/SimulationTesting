
package com.ma5951.utils.RobotControl.StatesTypes;

import com.ma5951.utils.RobotControl.Subsystems.StateControlledSubsystem;

public class SubsystemState extends State{

    private StateControlledSubsystem subsystem;

    public SubsystemState(String stateName , StateControlledSubsystem stateSubsystem) {
        super(stateName);
        subsystem = stateSubsystem;
    }

    public StateControlledSubsystem getSubsystem() {
        return subsystem;
    }

    public void setState() {
        subsystem.setTargetState(stateName); 
    }



}

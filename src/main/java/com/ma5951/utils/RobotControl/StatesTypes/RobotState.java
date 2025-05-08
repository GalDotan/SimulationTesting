
package com.ma5951.utils.RobotControl.StatesTypes;

public class RobotState extends State{

    private SubsystemState[] subsystemStatesArr;
    private Runnable runable;

    public RobotState(SubsystemState... subsystemStates) {
        super();
        String className = this.getClass().getName();
        setStateName(className);
        int i = 0;
        for (@SuppressWarnings("unused") SubsystemState subsystemState : subsystemStates) {
            i++;
        }
        subsystemStatesArr = new SubsystemState[i];
        i = 0;
        for (SubsystemState subsystemState : subsystemStates) {
            subsystemStatesArr[i] = subsystemState;
            i++;
        }

    }

    public RobotState(String stateName , Runnable toRun ,SubsystemState... subsystemStates) {
        super(stateName);
        runable = toRun;
    }

    public void setState() {
        runable.run();
        for (SubsystemState subsystemState : subsystemStatesArr) {
            subsystemState.setState();
        }
    }



}

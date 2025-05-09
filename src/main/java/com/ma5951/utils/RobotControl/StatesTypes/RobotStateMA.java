
package com.ma5951.utils.RobotControl.StatesTypes;

public class RobotStateMA extends State{

    private SubsystemState[] subsystemStatesArr;
    private Runnable runable;

    public RobotStateMA(String stateName , SubsystemState... subsystemStates) {
        super(stateName);
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

        runable = () -> {};

    }

    public RobotStateMA(String stateName , Runnable toRun ,SubsystemState... subsystemStates) {
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


package com.ma5951.utils.RobotControl.StatesTypes;

public class State {

    protected String stateName;

    public State(String stateName) {
        this.stateName = stateName;
    }

    public State() {

    }

    public void setStateName(String stateName) {
        this.stateName = stateName;
    }

    public String getStateName() {
        return stateName;
    }


}

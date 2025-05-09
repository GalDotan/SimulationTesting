
package com.ma5951.utils.RobotControl.StatesTypes;

import java.util.function.Supplier;

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

    public void setStateName(Supplier<String> stateName) {
        this.stateName = stateName.get();
    }

    public String getStateName() {
        return stateName;
    }


}

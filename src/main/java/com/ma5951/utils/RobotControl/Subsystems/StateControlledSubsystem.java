
package com.ma5951.utils.RobotControl.Subsystems;

import com.ma5951.utils.DashBoard.MAShuffleboard;
import com.ma5951.utils.Logger.MALog;
import com.ma5951.utils.RobotControl.StatesTypes.StatesConstants;
import com.ma5951.utils.RobotControl.StatesTypes.SystemFunctionState;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class StateControlledSubsystem extends SubsystemBase {

    private boolean systemCanMove = true;
    private SystemFunctionState systemFunctionState = StatesConstants.AUTOMATIC;
    private String targetState;
    private String lastState;
    private String systemName;
    protected MAShuffleboard board;

    public StateControlledSubsystem(String name) {
        targetState = StatesConstants.IDLE.getStateName();
        board = new MAShuffleboard(name);
        systemName = name;
        board.addBoolean(systemName + " Manuel", false);

    }

    public void setSystemFunctionState(SystemFunctionState FunctioState) {
        if (FunctioState == StatesConstants.MANUEL) {
            board.addBoolean(systemName + " Manuel", true);
        } else {
            board.addBoolean(systemName + " Manuel", false);
        }

        systemFunctionState = FunctioState;
    }

    public SystemFunctionState getSystemFunctionState() {
        return systemFunctionState;
    }

    public String getLastState() {
        return lastState;
    }

    public void setTargetState(String stateName) {
        lastState = targetState;
        targetState = stateName;
    }

    public boolean canMove() {
        return systemCanMove;
    }

    public String getCurrenState() {
        return targetState;
    }

    @Override
    public void periodic() {
        MALog.log("/RobotControl/" + systemName + "/Current State", getCurrenState());
        MALog.log("/RobotControl/" + systemName + "/System Function State", getSystemFunctionState().getStateName());
        MALog.log("/RobotControl/" + systemName + "/Can Move", canMove());

        if (board.getBoolean(systemName + " Manuel")) {
            setSystemFunctionState(StatesConstants.MANUEL);
        } else {
            setSystemFunctionState(StatesConstants.AUTOMATIC);
        } // TODO
    }

}

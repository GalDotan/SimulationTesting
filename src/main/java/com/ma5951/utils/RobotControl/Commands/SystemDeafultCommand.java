
package com.ma5951.utils.RobotControl.Commands;

import com.ma5951.utils.RobotControl.Subsystems.StateControlledSubsystem;


public abstract class SystemDeafultCommand {

    protected StateControlledSubsystem subsystem;

    public SystemDeafultCommand(StateControlledSubsystem subsystem) {
        this.subsystem = subsystem;
    }

    public StateControlledSubsystem getSubsystem() {
        return subsystem;
    }

    public abstract void Automatic();

    public abstract void Manual();

    public abstract void CantMove();

    public void CanMove() {
        Automatic();
    }

    public void Auto() {
        Automatic();
    }

    public void Test() {

    }

}

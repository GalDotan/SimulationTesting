
package com.ma5951.utils.RobotControl.Commands;

import com.ma5951.utils.RobotControl.Subsystems.StateControlledSubsystem;


public class SystemDeafultCommand {

    protected StateControlledSubsystem subsystem;

    public SystemDeafultCommand(StateControlledSubsystem subsystem) {
        this.subsystem = subsystem;
    }

    public StateControlledSubsystem getSubsystem() {
        return subsystem;
    }

    public void Automatic() {
        System.out.println("Automatic loop");
    }

    public void Manual() {

    }

    public void CantMove() {

    }

    public void CanMove() {
        Automatic();
    }

    public void Auto() {
        Automatic();
    }

    public void Test() {

    }

}

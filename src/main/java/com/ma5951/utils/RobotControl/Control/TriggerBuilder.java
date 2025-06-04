
package com.ma5951.utils.RobotControl.Control;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ma5951.utils.RobotControl.StatesTypes.RobotOporationState;
import com.ma5951.utils.RobotControl.StatesTypes.RobotStateMA;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TriggerManeger {

    private Supplier<RobotStateMA> worksInState;
    private Supplier<RobotOporationState> workInMode;

    public TriggerManeger(Supplier<RobotStateMA> worksInState, Supplier<RobotOporationState> workInMode) {
        this.worksInState = worksInState;
        this.workInMode = workInMode;
    }

    public void add(RobotOporationState workInMode , RobotStateMA workInState ,BooleanSupplier condition , Runnable action) {
        new Trigger(() -> workInMode == this.workInMode.get() && workInState == this.worksInState.get() && condition.getAsBoolean()).onTrue(
            new InstantCommand(action)
        );
    }

    public void add(RobotStateMA workInState ,BooleanSupplier condition , Runnable action) {
        new Trigger(() ->workInState == this.worksInState.get() && condition.getAsBoolean()).onTrue(
            new InstantCommand(action)
        );
    }

    public void add(RobotOporationState workInMode ,BooleanSupplier condition , Runnable action) {
        new Trigger(() -> workInMode == this.workInMode.get() && condition.getAsBoolean()).onTrue(
            new InstantCommand(action)
        );
    }

    public void add(BooleanSupplier condition , Runnable action) {
        new Trigger(condition).onTrue(
            new InstantCommand(action)
        );
    }





}

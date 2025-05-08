
package com.ma5951.utils.RobotControlAdv;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ma5951.utils.RobotControl.StatesTypes.State;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TriggerManeger {

    private Supplier<State> worksInState;
    private Supplier<State> workInMode;

    public TriggerManeger(Supplier<State> worksInState, Supplier<State> workInMode) {
        this.worksInState = worksInState;
        this.workInMode = workInMode;
    }

    public void add(State workInMode , State workInState ,BooleanSupplier condition , Runnable action) {
        new Trigger(() -> workInMode == this.workInMode.get() && workInState == this.worksInState.get() && condition.getAsBoolean()).onTrue(
            new InstantCommand(action)
        );
    }

    public void add(State workInMode ,BooleanSupplier condition , Runnable action) {
        new Trigger(() -> workInMode == this.workInMode.get() && condition.getAsBoolean()).onTrue(
            new InstantCommand(action)
        );
    }

    public void add(Boolean condition , Runnable action) {
        new Trigger(() -> condition).onTrue(
            new InstantCommand(action)
        );
    }





}

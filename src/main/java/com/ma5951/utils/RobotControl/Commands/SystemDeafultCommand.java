
package com.ma5951.utils.RobotControl.Commands;

import com.ma5951.utils.RobotControl.StatesTypes.StatesConstants;
import com.ma5951.utils.RobotControl.Subsystems.StateControlledSubsystem;

import edu.wpi.first.wpilibj2.command.Command;


public abstract class SystemDeafultCommand extends Command {

    protected StateControlledSubsystem subsystem;

    public SystemDeafultCommand(StateControlledSubsystem subsystem) {
        super();
        this.subsystem = subsystem;

        addRequirements(subsystem);
    }

    public StateControlledSubsystem getCommandSubsystem() {
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


    @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (subsystem.canMove()) {
      switch (StatesConstants.getRobotState().getStateName()) {
        case "TELEOP":
          switch (subsystem.getSystemFunctionState().getStateName()) {
            case "AUTOMATIC":
              Automatic();
              break;

            default:
              Manual();
              break;
          }
          break;
        case "AUTO":
          Auto();
          break;
        case "TEST":
          Test();
          break;
        default:
          CantMove();
          break;
      }
    } else {
      CantMove();
    }

  }

  @Override
  public void end(boolean interrupted) {
    CantMove();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}

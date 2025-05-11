
package com.ma5951.utils.RobotControl.Commands;

import com.ma5951.utils.RobotControl.StatesTypes.StatesConstants;
import com.ma5951.utils.RobotControl.Subsystems.StateControlledSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class DeafultCommandBuilder extends Command {

  private SystemDeafultCommand systemDeafultCommand;
  private StateControlledSubsystem subsystem;

  public DeafultCommandBuilder(SystemDeafultCommand command) {
    subsystem = command.getSubsystem();
    systemDeafultCommand = command;

    addRequirements(subsystem);
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
              systemDeafultCommand.Automatic();
              break;

            default:
              systemDeafultCommand.Manual();
              break;
          }
          break;
        case "AUTO":
          systemDeafultCommand.Auto();
          break;
        case "TEST":
          systemDeafultCommand.Test();
          break;
        default:
          systemDeafultCommand.CantMove();
          break;
      }
    } else {
      systemDeafultCommand.CantMove();
    }

  }

  @Override
  public void end(boolean interrupted) {
    systemDeafultCommand.CantMove();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}


package frc.robot.commands.Gripper;

import com.ma5951.utils.RobotControl.Commands.SystemDeafultCommand;

import frc.robot.RobotContainer;
import frc.robot.Subsystem.Gripper.Gripper;

public class GripperCommand extends SystemDeafultCommand {
    private static Gripper gripper = RobotContainer.gripper;

    public GripperCommand() {
        super(gripper);

    }

    @Override
    public void Automatic() {
        switch (gripper.getCurrenState()) {
            case "IDLE":
                gripper.setVoltage(0);
                break;
            case "HANDOFF":
                gripper.setVoltage(4);
                break;
            case "CORAL_INTAKE":
                gripper.setVoltage(0);
                break;
            case "L1_SCORING":
                gripper.setVoltage(0);
                break;
            case "HOLD":
                gripper.setVoltage(0.5);
                break;
            case "SCORING":
                gripper.setVoltage(-4);

                break;
        }
    }

    @Override
    public void Manual() {
        
    }

    @Override
    public void CantMove() {
        gripper.setVoltage(0);
    }
}
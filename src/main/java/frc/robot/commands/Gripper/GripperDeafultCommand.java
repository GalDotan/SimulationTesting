
package frc.robot.commands.Gripper;

import com.ma5951.utils.RobotControl.Commands.RobotFunctionStatesCommand;

import frc.robot.RobotContainer;
import frc.robot.Subsystem.Gripper.Gripper;

public class GripperDeafultCommand extends RobotFunctionStatesCommand {
    private static Gripper gripper = RobotContainer.gripper;

    public GripperDeafultCommand() {
        super(gripper);

        addRequirements(gripper);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        gripper.setVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void AutomaticLoop() {
        super.AutomaticLoop();
        switch (gripper.getTargetState().getName()) {
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
        }
    }

    @Override
    public void CAN_MOVE() {
        super.CAN_MOVE();
    }

    @Override
    public void CANT_MOVE() {
        super.CANT_MOVE();
        gripper.setVoltage(0);
    }

    @Override
    public void ManuelLoop() {
        super.ManuelLoop();
    }

    @Override
    public void AutoLoop() {
        super.AutoLoop();
        AutomaticLoop();
    }

    @Override
    public void TestLoop() {
        super.TestLoop();
    }
}
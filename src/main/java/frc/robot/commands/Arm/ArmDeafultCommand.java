
package frc.robot.commands.Arm;

import com.ma5951.utils.Logger.MALog;
import com.ma5951.utils.RobotControl.Commands.RobotFunctionStatesCommand;

import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystem.Arm.Arm;

public class ArmDeafultCommand extends RobotFunctionStatesCommand {
    private static Arm arm = RobotContainer.arm;

    public ArmDeafultCommand() {
        super(arm);

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        MALog.log("/Tuning/Arm Angle", 0);
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        arm.setVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void AutomaticLoop() {
        super.AutomaticLoop();
        switch (arm.getCurrenState().getStateName()) {
            case "IDLE":
                arm.setAngle(0);
                break;
            case "HOLD":
                arm.setAngle(0);
                break;
            case "INTAKE":
                arm.setAngle(0);
                break;
            case "HANDOFF":
                arm.setAngle(0);
                break;
            case "SCORING":
                arm.setAngle(SuperStructure.getScoringLevel().scoringAngle);
                break;
            case "PRE_SCORING":
                arm.setAngle(SuperStructure.getScoringLevel().preAngle);
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
        arm.setVoltage(0);
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
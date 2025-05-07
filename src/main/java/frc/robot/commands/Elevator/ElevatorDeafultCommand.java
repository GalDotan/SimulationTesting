
package frc.robot.commands.Elevator;

import com.ma5951.utils.Logger.MALog;
import com.ma5951.utils.RobotControl.Commands.RobotFunctionStatesCommand;

import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystem.Elevator.Elevator;

public class ElevatorDeafultCommand extends RobotFunctionStatesCommand {
    private static Elevator elevator = RobotContainer.elevator;

    public ElevatorDeafultCommand() {
        super(elevator);

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        MALog.log("/Tuning/Elevator Hight", 0);
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void AutomaticLoop() {
        super.AutomaticLoop();
        switch (elevator.getTargetState().getName()) {
            case "IDLE":
                elevator.setHight(1.2);
                break;
            case "HOLD":
                elevator.setHight(1.2);
                break;
            case "INTAKE":
                elevator.setHight(1.2);
                break;
            case "SCORING":
                elevator.setHight(SuperStructure.getScoringLevel().scoringHight);
                break;
            case "PRE_SCORING":
                elevator.setHight(SuperStructure.getScoringLevel().preHight);
                break;
            case "HANDOFF":
                elevator.setHight(1.05);
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
        elevator.setVoltage(0);
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
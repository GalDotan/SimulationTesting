
package frc.robot.commands.Elevator;

import com.ma5951.utils.RobotControl.Commands.SystemDeafultCommand;

import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystem.Elevator.Elevator;

public class ElevatorCommand extends SystemDeafultCommand {
    private static Elevator elevator = RobotContainer.elevator;

    public ElevatorCommand() {
        super(elevator);

    }

    @Override
    public void Automatic() {
        switch (elevator.getCurrenState()) {
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
    public void Manual() {

    }

    @Override
    public void CantMove() {
        elevator.setVoltage(0);
    }
}
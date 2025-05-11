
package frc.robot.commands.Arm;

import com.ma5951.utils.RobotControl.Commands.SystemDeafultCommand;

import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystem.Arm.Arm;

public class ArmCommand extends SystemDeafultCommand {
    private static Arm arm = RobotContainer.arm;

    public ArmCommand() {
        super(arm);

    }

    @Override
    public void Automatic() {
        switch (arm.getCurrenState()) {
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

        };
    }

    @Override
    public void Manual() {
        
    }

    @Override
    public void CantMove() {
        arm.setVoltage(0);;
    }
}
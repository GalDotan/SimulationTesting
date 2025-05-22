
package frc.robot.commands.Intake;

import com.ma5951.utils.RobotControl.Commands.SystemDeafultCommand;

import frc.robot.RobotContainer;
import frc.robot.Subsystem.Intake.IntakeArm.IntakeArm;

public class IntakeArmCommand extends SystemDeafultCommand {
    private static IntakeArm intakeArm = RobotContainer.intakeArm;

    public IntakeArmCommand() {
        super(intakeArm);

    }

    @Override
    public void Automatic() {
        switch (intakeArm.getCurrenState()) {
            case "IDLE":
                intakeArm.setArmAngle(112);
                break;
            case "HANDOFF":
                intakeArm.setArmAngle(126);
                break;
            case "CORAL_INTAKE":
                intakeArm.setArmAngle(0);
                break;
            case "L1_SCORING":
                intakeArm.setArmAngle(105);
                break;
        }
    }

    @Override
    public void Manual() {
       
    }

    @Override
    public void CantMove() {
        intakeArm.setVoltage(0);
    }
}
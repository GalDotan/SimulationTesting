
package frc.robot.commands.Intake;

import com.ma5951.utils.RobotControl.Commands.SystemDeafultCommand;

import frc.robot.RobotContainer;
import frc.robot.Subsystem.Intake.IntakeRoller.IntakeRoller;

public class IntakeRollerCommand extends SystemDeafultCommand {
    private static IntakeRoller intakeRoller = RobotContainer.intakeRoller;

    public IntakeRollerCommand() {
        super(intakeRoller);

    }

    @Override
    public void Automatic() {
        switch (intakeRoller.getCurrenState()) {
            case "IDLE":
                intakeRoller.setVoltage(0);
                break;
            case "HANDOFF":
                intakeRoller.setVoltage(-4);
                break;
            case "CORAL_INTAKE":
                intakeRoller.setVoltage(4);
                break;
            case "L1_SCORING":
                intakeRoller.setVoltage(-3);
                break;
        }
    }

    @Override
    public void Manual() {

    }


    @Override
    public void CantMove() {
        intakeRoller.setVoltage(0);
    }
}
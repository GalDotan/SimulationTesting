
package frc.robot.commands.Intake;

import com.ma5951.utils.RobotControl.Commands.RobotFunctionStatesCommand;

import frc.robot.RobotContainer;
import frc.robot.Subsystem.Intake.IntakeRoller.IntakeRoller;

public class IntakeRollerDeafultCommand extends RobotFunctionStatesCommand {
    private static IntakeRoller intakeRoller = RobotContainer.intakeRoller;

    public IntakeRollerDeafultCommand() {
        super(intakeRoller);

        addRequirements(intakeRoller);
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
        intakeRoller.setVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void AutomaticLoop() {
        super.AutomaticLoop();
        switch (intakeRoller.getTargetState().getName()) {
            case "IDLE":
                intakeRoller.setVoltage(0);
                break;
            case "HANDOFF":
                intakeRoller.setVoltage(-4);
                break;
            case "CORAL_INTAKE":
                intakeRoller.setVoltage(6);
                break;
            case "L1_SCORING":
                intakeRoller.setVoltage(-3);
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
        intakeRoller.setVoltage(0);
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

package frc.robot.commands.Intake;

import com.ma5951.utils.RobotControl.Commands.RobotFunctionStatesCommand;

import frc.robot.RobotContainer;
import frc.robot.Subsystem.Intake.IntakeArm.IntakeArm;

public class IntakeArmDeafultCommand extends RobotFunctionStatesCommand {
    private static IntakeArm intakeArm = RobotContainer.intakeArm;

    public IntakeArmDeafultCommand() {
        super(intakeArm);

        addRequirements(intakeArm);
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
        intakeArm.setVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void AutomaticLoop() {
        super.AutomaticLoop();
        switch (intakeArm.getTargetState().getName()) {
            case "IDLE":
                intakeArm.setArmAngle(120);
                break;
            case "HANDOFF":
                intakeArm.setArmAngle(120);
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
    public void CAN_MOVE() {
        super.CAN_MOVE();
    }

    @Override
    public void CANT_MOVE() {
        super.CANT_MOVE();
        intakeArm.setVoltage(0);
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
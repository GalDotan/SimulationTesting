
package frc.robot.commands.Arm;

import com.ma5951.utils.RobotControl.Commands.RobotFunctionStatesCommand;

import frc.robot.RobotContainer;
import frc.robot.Subsystem.Arm.Arm;

public class ArmDeafultCommand extends RobotFunctionStatesCommand {
    private static Arm arm = RobotContainer.arm;

    public ArmDeafultCommand() {
        super(arm);

        addRequirements(arm);
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
         arm.setVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void AutomaticLoop() {
        super.AutomaticLoop();
        switch (arm.getTargetState().getName()) {
            case "IDLE":
                arm.setAngle(0);
                break;
            case "HOLD":
                arm.setAngle(90);
                break;
            case "INTAKE":
                arm.setAngle(0);
                break;
            case "SCORING":
                arm.setAngle(100);
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
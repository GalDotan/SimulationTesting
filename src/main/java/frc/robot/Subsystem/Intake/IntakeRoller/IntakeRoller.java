
package frc.robot.Subsystem.Intake.IntakeRoller;

import com.ma5951.utils.RobotControl.Subsystems.StateControlledSubsystem;

import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystem.Intake.IntakeRoller.IOs.IntakeRollerIO;
import frc.robot.Subsystem.Intake.IntakeRoller.IOs.IntakeRollerIOReal;
import frc.robot.Subsystem.Intake.IntakeRoller.IOs.IntakeRollerIOSim;


public class IntakeRoller extends StateControlledSubsystem{
    private static IntakeRoller intake;

    private IntakeRollerIO intakeIO ;

    private IntakeRoller() {
        super("Intake Roller");
        intakeIO = getRollerIntakeIO();

        intakeIO.setNeutralMode(true);
    }

    public static IntakeRollerIO getRollerIntakeIO() {
        if (Robot.isReal()) {
            return new IntakeRollerIOReal();
        } else {
            return new IntakeRollerIOSim();
        }
    }

    public boolean hasCoral() {
        return intakeIO.hasCoral();
    }

    public double getCurrent() {
        return intakeIO.getCurrent();
    }

    public double getVelocity() {
        return intakeIO.getVelocity();
    }

    public double getVolts() {
        return intakeIO.getAppliedVolts();
    }

    public void setVoltage(double voltage) {
        intakeIO.setVoltage(voltage);
    }

    public boolean IntakeCanMove() {
        return RobotContainer.currentRobotState == RobotConstants.INTAKE && RobotContainer.intakeArm.AtPoint() && !SuperStructure.hasGamePiece();
    }

    public boolean HandOffCanMove() {
        return RobotContainer.currentRobotState == RobotConstants.HANDOFF && RobotContainer.intakeArm.AtPoint() && RobotContainer.arm.AtPoint() && hasCoral();
    }

    @Override
    public boolean canMove() {
        return IntakeCanMove() || HandOffCanMove();
    }

    public static IntakeRoller getInstance() {
        if (intake == null) {
            intake = new IntakeRoller();
        }
        return intake;
    }
    
    @Override
    public void periodic() {
        super.periodic();
        intakeIO.updatePeriodic();
        
    }


}

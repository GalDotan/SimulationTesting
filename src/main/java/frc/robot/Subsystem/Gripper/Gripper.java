
package frc.robot.Subsystem.Gripper;

import com.ma5951.utils.RobotControl.Subsystems.StateControlledSubsystem;

import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.Subsystem.Gripper.IOs.GripperIO;
import frc.robot.Subsystem.Gripper.IOs.GripperIOReal;
import frc.robot.Subsystem.Gripper.IOs.GripperIOSim;


public class Gripper extends StateControlledSubsystem{
    private static Gripper gripper;

    private GripperIO gripperIO;

    private Gripper() {
        super("Gripper");
        gripperIO = getGripperIO();

        gripperIO.setNeutralMode(true);
    }

    public static GripperIO getGripperIO() {
        if (Robot.isReal()) {
            return new GripperIOReal();
        } else {
            return new GripperIOSim();
        }
    }

    public boolean hasCoral() {
        return gripperIO.hasCoral();
    }

    public double getCurrent() {
        return gripperIO.getCurrent();
    }

    public double getVelocity() {
        return gripperIO.getVelocity();
    }

    public double getVolts() {
        return gripperIO.getAppliedVolts();
    }

    public void setVoltage(double voltage) {
        gripperIO.setVoltage(voltage);
    }

    public boolean HandOffCanMove() {
        return RobotContainer.currentRobotState == RobotConstants.HANDOFF && RobotContainer.intakeArm.AtPoint() && RobotContainer.arm.AtPoint() && !hasCoral();
    }

    public boolean ScoringCanMove() {
        return RobotContainer.currentRobotState == RobotConstants.SCORING;
    }

    @Override
    public boolean canMove() {
        return HandOffCanMove() || ScoringCanMove();
    }

    public static Gripper getInstance() {
        if (gripper == null) {
            gripper = new Gripper();
        }
        return gripper;
    }
    
    @Override
    public void periodic() {
        super.periodic();
        gripperIO.updatePeriodic();

        
    }


}

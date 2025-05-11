
package frc.robot.Subsystem.Intake.IntakeRoller;

import com.ma5951.utils.RobotControl.StatesTypes.SubsystemState;

import frc.robot.Robot;
import frc.robot.Subsystem.Intake.IntakeRoller.IOs.IntakeRollerIO;
import frc.robot.Subsystem.Intake.IntakeRoller.IOs.IntakeRollerIOReal;
import frc.robot.Subsystem.Intake.IntakeRoller.IOs.IntakeRollerIOSim;

public class IntakeRollerConstants {

    public static final double GEAR_RATIO = 3;


    public static final double STATOR_CURRENT_LIMIT = 40;
    public static final boolean ENABLE_CURRENT_LIMIT = true;

    public static final double k_CAN_MOVE_CURRENT_LIMIT = 60;


    public static final SubsystemState IDLE = new SubsystemState("IDLE", IntakeRoller.getInstance());
    public static final SubsystemState HANDOFF = new SubsystemState("HANDOFF", IntakeRoller.getInstance());
    public static final SubsystemState CORAL_INTAKE = new SubsystemState("CORAL_INTAKE", IntakeRoller.getInstance());
    public static final SubsystemState ALGE_INTAKE = new SubsystemState("ALGE_INTAKE", IntakeRoller.getInstance());
    public static final SubsystemState L1_SCORING = new SubsystemState("L1_SCORING", IntakeRoller.getInstance());

    public static IntakeRollerIO getRollerIntakeIO() {
        if (Robot.isReal()) {
            return new IntakeRollerIOReal();
        } else {
            return new IntakeRollerIOSim();
        }
    }



}

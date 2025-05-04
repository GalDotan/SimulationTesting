
package frc.robot.Subsystem.Intake.IntakeRoller;

import com.ma5951.utils.RobotControl.StatesTypes.State;
import com.ma5951.utils.RobotControl.StatesTypes.StatesConstants;

import frc.robot.Robot;
import frc.robot.Subsystem.Intake.IntakeRoller.IOs.IntakeRollerIO;
import frc.robot.Subsystem.Intake.IntakeRoller.IOs.IntakeRollerIOReal;
import frc.robot.Subsystem.Intake.IntakeRoller.IOs.IntakeRollerIOSim;

public class IntakeRollerConstants {

    public static final double GEAR_RATIO = 5;


    public static final double STATOR_CURRENT_LIMIT = 40;
    public static final boolean ENABLE_CURRENT_LIMIT = true;

    public static final double k_CAN_MOVE_CURRENT_LIMIT = 60;


    public static final State IDLE = StatesConstants.IDLE;
    public static final State HANDOFF = new State("HANDOFF");
    public static final State CORAL_INTAKE = new State("CORAL_INTAKE");
    public static final State ALGE_INTAKE = new State("CORAL_INTAKE");
    public static final State L1_SCORING = new State("L1_SCORING");

    public static final State[] SUBSYSTEM_STATES = new State[] {IDLE, HANDOFF, CORAL_INTAKE, ALGE_INTAKE, L1_SCORING};

    public static IntakeRollerIO getRollerIntakeIO() {
        if (Robot.isReal()) {
            return new IntakeRollerIOReal();
        } else {
            return new IntakeRollerIOSim();
        }
    }



}

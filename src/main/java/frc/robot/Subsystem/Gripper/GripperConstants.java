
package frc.robot.Subsystem.Gripper;

import com.ma5951.utils.RobotControl.StatesTypes.State;
import com.ma5951.utils.RobotControl.StatesTypes.StatesConstants;

import frc.robot.Robot;
import frc.robot.Subsystem.Gripper.IOs.GripperIO;
import frc.robot.Subsystem.Gripper.IOs.GripperIOReal;
import frc.robot.Subsystem.Gripper.IOs.GripperIOSim;

public class GripperConstants {

    public static final double GEAR_RATIO = 5;


    public static final double STATOR_CURRENT_LIMIT = 40;
    public static final boolean ENABLE_CURRENT_LIMIT = true;

    public static final double k_CAN_MOVE_CURRENT_LIMIT = 60;


    public static final State IDLE = StatesConstants.IDLE;
    public static final State HANDOFF = new State("HANDOFF");
    public static final State HOLD = new State("HOLD");
    public static final State CORAL_INTAKE = new State("CORAL_INTAKE");
    public static final State ALGE_INTAKE = new State("CORAL_INTAKE");
    public static final State SCORING = new State("SCORING");

    public static final State[] SUBSYSTEM_STATES = new State[] {IDLE, HANDOFF, CORAL_INTAKE, ALGE_INTAKE,HOLD,SCORING};

    public static GripperIO getGripperIO() {
        if (Robot.isReal()) {
            return new GripperIOReal();
        } else {
            return new GripperIOSim();
        }
    }



}


package frc.robot.Subsystem.Gripper;

import com.ma5951.utils.RobotControl.StatesTypes.SubsystemState;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Subsystem.Gripper.IOs.GripperIO;
import frc.robot.Subsystem.Gripper.IOs.GripperIOReal;
import frc.robot.Subsystem.Gripper.IOs.GripperIOSim;

public class GripperConstants {

    public static final double GEAR_RATIO = 5;


    public static final double STATOR_CURRENT_LIMIT = 40;
    public static final boolean ENABLE_CURRENT_LIMIT = true;

    public static final double k_CAN_MOVE_CURRENT_LIMIT = 60;

    public static final SubsystemState IDLE = new SubsystemState("IDLE", Gripper.getInstance());
    public static final SubsystemState HANDOFF = new SubsystemState("HANDOFF", Gripper.getInstance());
    public static final SubsystemState CORAL_INTAKE = new SubsystemState("CORAL_INTAKE", Gripper.getInstance());
    public static final SubsystemState ALGE_INTAKE = new SubsystemState("ALGE_INTAKE", Gripper.getInstance());
    public static final SubsystemState SCORING = new SubsystemState("SCORING", Gripper.getInstance());
    public static final SubsystemState HOLD = new SubsystemState("HOLD", Gripper.getInstance());


    public static GripperIO getGripperIO() {
        if (Robot.isReal()) {
            return new GripperIOReal();
        } else {
            return new GripperIOSim();
        }
    }



}

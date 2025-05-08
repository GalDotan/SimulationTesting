
package frc.robot;

import com.ma5951.utils.RobotControl.StatesTypes.State;

import frc.robot.Subsystem.Gripper.Gripper;

public interface GlobalConstants {

    public static final State IDLE = RobotConstants.IDLE;
    public static final State INTAKE = RobotConstants.INTAKE;
    public static final State SCORING = RobotConstants.SCORING;
    public static final State HANDOFF = RobotConstants.HANDOFF;
    public static final State PRE_SCORING = RobotConstants.PRE_SCORING;
    public static final State HOLD = RobotConstants.HOLD;

    public static final Gripper gripper = Gripper.getInstance();

}

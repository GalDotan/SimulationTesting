
package frc.robot.Subsystem.Intake.IntakeArm;

import com.ma5951.utils.RobotControl.StatesTypes.SubsystemState;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Subsystem.Intake.IntakeArm.IOs.IntakeArmIO;
import frc.robot.Subsystem.Intake.IntakeArm.IOs.IntakeArmIOReal;
import frc.robot.Subsystem.Intake.IntakeArm.IOs.IntakeArmIOSim;

public class IntakeArmConstants {

    public static final double MAX_ANGLE = 127; 
    public static final double MIN_ANGLE = 0;  

    public static final double INTAKE_CORALS_ANGLE = 26;
    public static final double L1_SCORE_ANGLE = 26;
    public static final double HANDOFF_ANGLE = 26;

    public static final double kP = 15;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double FEED_FORWARD_VOLTAGE = 0.13;
    public static final double ARM_GEAR_RATIO = 10;

    public static final int CONTROL_SLOT = 0;
    public static final double TOLERANCE = 2;

    public static final double ARM_STATOR_CURRENT_LIMIT = 25;
    public static final boolean ARM_ENABLE_CURRENT_LIMIT = true;


    public static final double k_CAN_MOVE_CURRENT_LIMIT = 60;

    public static final Pose3d SIM_INTAKE_OFFSET = new Pose3d(new Translation3d(0.25,0,0.175), new Rotation3d(0,0,0));

    public static final SubsystemState IDLE = new SubsystemState("IDLE", IntakeArm.getInstance());
    public static final SubsystemState HANDOFF = new SubsystemState("HANDOFF", IntakeArm.getInstance());
    public static final SubsystemState CORAL_INTAKE = new SubsystemState("CORAL_INTAKE", IntakeArm.getInstance());
    public static final SubsystemState ALGE_INTAKE = new SubsystemState("ALGE_INTAKE", IntakeArm.getInstance());
    public static final SubsystemState L1_SCORING = new SubsystemState("L1_SCORING", IntakeArm.getInstance());

    



}

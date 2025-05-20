package frc.robot.Subsystem.Arm;

import com.ma5951.utils.RobotControl.StatesTypes.SubsystemState;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Subsystem.Arm.IOs.ArmIO;
import frc.robot.Subsystem.Arm.IOs.ArmIOReal;
import frc.robot.Subsystem.Arm.IOs.ArmIOSim;

public class ArmConstants {

    public static final double MAX_ANGLE = 360; 
    public static final double MIN_ANGLE = 0;  
    public static final double PARALLEL_ANGLE = 90;

    public static final double ENDEFFECTOR_RADIUS = 0.66483;
    public static final double INTAKE_COSTRAINT_RADIUS = 0.38713;



    public static final double PRE_ANGLE_L2 = 120;
    public static final double PRE_ANGLE_L3 = 120;
    public static final double PRE_ANGLE_L4 = 120;

    public static final double SCORING_ANGLE_L2 = PRE_ANGLE_L2 - 5;
    public static final double SCORING_ANGLE_L3 = PRE_ANGLE_L3 - 5;
    public static final double SCORING_ANGLE_L4 = PRE_ANGLE_L4 - 10;



    public static final double INTAKE_CORALS_ANGLE = 26;
    public static final double EJECT_BALL_START_ANGLE = 0;
    public static final double EJECT_BALL_STOP_ANGLE = 125;

    public static final double SKYHOOK_START_ANGLE = 99;
    public static final double SKYHOOK_START_HIGHT = 0.7;
    public static final double SKYHOOK_END_ANGLE = 180;
    
    public static final double HOLD_ANGLE = 145;
    public static final double ABS_ENCODER_OFFSET = 0;

    public static final double GEAR_RATIO = 20.4;
    public static final double CANCODER_RATIO = 3.4;

    public static final double kP = 20;//35
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double FEED_FORWARD_VOLTAGE = 0.085;

    public static final int CONTROL_SLOT = 0;
    public static final double TOLERANCE = 2;

    public static final double PEAK_CURRENT_LIMIT = 60;
    public static final double CONTINUOUS_CURRENT_LIMIT = 20;
    public static final double CONTINUOUS_CURRENT_DURATION = 0.1;
    public static final boolean ENABLE_CURRENT_LIMIT = true;

    public static final double kCAN_MOVE_CURRENT_LIMIT = 20000;
    public static final double kMANUEL_VOLTAGE_LIMIT = 7;


    public static final SubsystemState IDLE = new SubsystemState("IDLE", Arm.getInstance());
    public static final SubsystemState HOLD = new SubsystemState("HOLD", Arm.getInstance());
    public static final SubsystemState INTAKE = new SubsystemState("INTAKE", Arm.getInstance());
    public static final SubsystemState HANDOFF = new SubsystemState("HANDOFF", Arm.getInstance());
    public static final SubsystemState PRE_SCORING = new SubsystemState("PRE_SCORING", Arm.getInstance());
    public static final SubsystemState SCORING = new SubsystemState("SCORING", Arm.getInstance());
    public static final SubsystemState ALGE_GROUND = new SubsystemState("ALGE_GROUND", Arm.getInstance());
    public static final SubsystemState ALGE_REEF = new SubsystemState("ALGE_REEF", Arm.getInstance());
    public static final SubsystemState ALGE_NET = new SubsystemState("ALGE_REEF", Arm.getInstance());
    public static final SubsystemState ALGE_PROCESSOR = new SubsystemState("ALGE_PROCESSOR", Arm.getInstance());

    public static final Pose3d SIM_ARM_OFFSET = new Pose3d(new Translation3d(-0.124,0,1.262), new Rotation3d(0,0,0));

    
}

package frc.robot.Subsystem.Arm;

import com.ma5951.utils.RobotControl.StatesTypes.State;
import com.ma5951.utils.RobotControl.StatesTypes.StatesConstants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Robot;
import frc.robot.Subsystem.Arm.IOs.ArmIO;
import frc.robot.Subsystem.Arm.IOs.ArmIOReal;
import frc.robot.Subsystem.Arm.IOs.ArmIOSim;

public class ArmConstants {

    public static final double MAX_ANGLE = 360; 
    public static final double MIN_ANGLE = 0;  
    public static final double PARALLEL_ANGLE = 90;
    public static final double MAX_ANGLE_BALL = 120; 
    public static final double ZERO_ANGLE = 10;

    public static final double ANGLE_L1 = 24;
    public static final double ANGLE_L2 = 154;//140
    public static final double ANGLE_L3 = 140;
    public static final double ANGLE_L4 = 127;
    public static final double ANGLE_OFFSET_L4_AFTER_EJECT = 14;

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

    public static final double kP = 30;//35
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

    public static final State IDLE = StatesConstants.IDLE;
    public static final State HOLD = new State("HOLD");
    public static final State INTAKE = new State("INTAKE");
    public static final State HANDOFF = new State("HANDOFF");
    public static final State PRE_SCORING = new State("PRE_SCORING");
    public static final State SCORING = new State("SCORING");
    public static final State ALGE_GROUND = new State("ALGE_GROUND");
    public static final State ALGE_REEF = new State("ALGE_REEF");
    public static final State ALGE_NET = new State("ALGE_REEF");
    public static final State ALGE_PROCESSOR = new State("ALGE_PROCESSOR");
    

    public static final Pose3d SIM_ARM_OFFSET = new Pose3d(new Translation3d(-0.124,0,1.262), new Rotation3d(0,0,0));

    public static final State[] SUBSYSTEM_STATES = new State[] {IDLE, HOLD, INTAKE, HANDOFF, PRE_SCORING , SCORING , ALGE_GROUND , ALGE_REEF, ALGE_NET, ALGE_PROCESSOR};

    public static ArmIO getArmIO() {
        if (Robot.isReal()) {
            return new ArmIOReal();
        } else {
            return new ArmIOSim();
        }
    }
}

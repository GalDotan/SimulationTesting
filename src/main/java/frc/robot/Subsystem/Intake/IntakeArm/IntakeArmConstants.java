
package frc.robot.Subsystem.Intake.IntakeArm;

import com.ma5951.utils.RobotControl.StatesTypes.State;
import com.ma5951.utils.RobotControl.StatesTypes.StatesConstants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Robot;
import frc.robot.Subsystem.Intake.IntakeArm.IOs.IntakeArmIO;
import frc.robot.Subsystem.Intake.IntakeArm.IOs.IntakeArmIOReal;
import frc.robot.Subsystem.Intake.IntakeArm.IOs.IntakeArmIOSim;

public class IntakeArmConstants {

    public static final double MAX_ANGLE = 140; 
    public static final double MIN_ANGLE = 0;  

    public static final double INTAKE_CORALS_ANGLE = 26;
    public static final double L1_SCORE_ANGLE = 26;
    public static final double HANDOFF_ANGLE = 26;

    public static final double kP = 80;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double FEED_FORWARD_VOLTAGE = 0.085;
    public static final double ARM_GEAR_RATIO = 60;

    public static final int CONTROL_SLOT = 0;
    public static final double TOLERANCE = 2;

    public static final double ARM_STATOR_CURRENT_LIMIT = 25;
    public static final boolean ARM_ENABLE_CURRENT_LIMIT = true;


    public static final double k_CAN_MOVE_CURRENT_LIMIT = 60;

    public static final Pose3d SIM_INTAKE_OFFSET = new Pose3d(new Translation3d(0.25,0,0.175), new Rotation3d(0,0,0));

    public static final State IDLE = StatesConstants.IDLE;
    public static final State HANDOFF = new State("HANDOFF");
    public static final State CORAL_INTAKE = new State("CORAL_INTAKE");
    public static final State ALGE_INTAKE = new State("ALGE_INTAKE");
    public static final State L1_SCORING = new State("L1_SCORING");

    public static final State[] SUBSYSTEM_STATES = new State[] {IDLE, HANDOFF, CORAL_INTAKE, ALGE_INTAKE, L1_SCORING};

    public static IntakeArmIO getIntakeIO() {
        if (Robot.isReal()) {
            return new IntakeArmIOReal();
        } else {
            return new IntakeArmIOSim();
        }
    }



}

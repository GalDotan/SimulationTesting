
package frc.robot.Subsystem.Elevator;

import com.ma5951.utils.RobotControl.StatesTypes.State;
import com.ma5951.utils.RobotControl.StatesTypes.StatesConstants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Robot;
import frc.robot.Subsystem.Elevator.IOs.ElevatorIO;
import frc.robot.Subsystem.Elevator.IOs.ElevatorIOReal;
import frc.robot.Subsystem.Elevator.IOs.ElevatorIOSim;

public class ElevatorConstants {

    public static final double MAX_HIGHT = 1.50;
    public static final double MIN_HIGHT = -0.005;
    
    
    public static final double PRE_HIGHT_L2 = 0.48;
    public static final double PRE_HIGHT_L3 = 0.88;
    public static final double PRE_HIGHT_L4 = 1.5;


    public static final double SCORING_HIGHT_L2 = PRE_HIGHT_L2 - 0.08;
    public static final double SCORING_HIGHT_L3 = PRE_HIGHT_L3 - 0.15;
    public static final double SCORING_HIGHT_L4 = PRE_HIGHT_L4 - 0.2;




    public static final double HIGHT_INTAKE_CORAL = 0.565 - 0.04;//0.53
    public static final double HIGHT_EJECT_BALL_LOW =  0.23 - 0.01; //
    public static final double HIGHT_EJECT_BALL_HIGH = 0.6 - 0.02; // 0.7
    public static final double HIGHT_PROSESOR = 0;
    public static final double HIGHT_ZERO = 0;

    public static final double SKYHOOK_STOP_HIGHT = 1.3;
    public static final double SKYHOOK_VOLTAGE = 7;

    public static final double GEAR = (82d / 12d) /2d;
    public static final double SPROKET_PITCH_DIAMETER = 0.0444754;
    public static final double SPROKET_CIRCUMFERENCE = SPROKET_PITCH_DIAMETER * Math.PI;

    public static final int CONTROL_SLOT = 0;
    public static final double kP = 0.7;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double TOLORANCE = 0.01;

    public static final double CURRENT_LIMIT = 80;
    public static final double CONTINUOUS_LOWER_LIMIT = 80; 
    public static final double CONTINUOUS_CURRENT_DURATION = 0.1;
    public static final boolean ENABLE_CURRENT_LIMIT = false;

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
    

    public static final Pose3d SIM_ELEVATOR_OFFSET = new Pose3d(new Translation3d(0,0,0.15), new Rotation3d(0,0,0));

    public static final State[] SUBSYSTEM_STATES = new State[] {IDLE, HOLD, INTAKE, HANDOFF, PRE_SCORING , SCORING , ALGE_GROUND , ALGE_REEF, ALGE_NET, ALGE_PROCESSOR};


    public static final ElevatorIO getElevatorIO() {
        if (Robot.isReal()) {
            return new ElevatorIOReal();
        }

        return new ElevatorIOSim();
    }
 
}

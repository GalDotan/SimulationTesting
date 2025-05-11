
package frc.robot;

import com.ma5951.utils.RobotControl.StatesTypes.RobotStateMA;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Subsystem.Arm.ArmConstants;
import frc.robot.Subsystem.Elevator.ElevatorConstants;
import frc.robot.Subsystem.Gripper.GripperConstants;
import frc.robot.Subsystem.Intake.IntakeArm.IntakeArmConstants;
import frc.robot.Subsystem.Intake.IntakeRoller.IntakeRollerConstants;

public class RobotConstants {

    // Robot Constants
    public static final boolean COMP_LOG = false;
    public static final double kDELTA_TIME = 0.02;

    // Robot Control

    // States
    public static final RobotStateMA IDLE = new RobotStateMA(
            "IDLE",
            IntakeRollerConstants.IDLE,
            IntakeArmConstants.IDLE,
            GripperConstants.IDLE,
            ElevatorConstants.IDLE,
            ArmConstants.IDLE);
    public static final RobotStateMA INTAKE = new RobotStateMA(
            "INTAKE",
            IntakeRollerConstants.CORAL_INTAKE,
            IntakeArmConstants.CORAL_INTAKE,
            GripperConstants.IDLE,
            ElevatorConstants.INTAKE,
            ArmConstants.INTAKE);
    public static final RobotStateMA HANDOFF = new RobotStateMA(
            "HANDOFF",
            IntakeRollerConstants.HANDOFF,
            IntakeArmConstants.HANDOFF,
            GripperConstants.HANDOFF,
            ElevatorConstants.HANDOFF,
            ArmConstants.HANDOFF);
    public static final RobotStateMA HOLD = new RobotStateMA(
            "HOLD",
            IntakeRollerConstants.IDLE,
            IntakeArmConstants.IDLE,
            GripperConstants.HOLD,
            ElevatorConstants.HOLD,
            ArmConstants.HOLD);
    public static final RobotStateMA PRE_SCORING = new RobotStateMA(
            "PRE_SCORING",
            IntakeRollerConstants.IDLE,
            IntakeArmConstants.IDLE,
            GripperConstants.IDLE,
            ElevatorConstants.PRE_SCORING,
            ArmConstants.PRE_SCORING);
    public static final RobotStateMA SCORING = new RobotStateMA(
            "SCORING",
            IntakeRollerConstants.IDLE,
            IntakeArmConstants.IDLE,
            GripperConstants.SCORING,
            ElevatorConstants.SCORING,
            ArmConstants.SCORING);


    // FieldConstants
    public static final Translation2d FieldZeroCorner = new Translation2d(0, 0);
    public static final Translation2d FieldFarCorner = new Translation2d(16.58, 8.20);
    public static final Translation2d FieldMiddlePoint = new Translation2d(16.58 / 2, 8.20 / 2);

    // Reef
    public static final Pose2d Tag6Pose = new Pose2d(13.474446, 3.3063179999999996, Rotation2d.fromDegrees(120));
    public static final Pose2d Tag7Pose = new Pose2d(13.890498, 4.0259, Rotation2d.fromDegrees(180));
    public static final Pose2d Tag8Pose = new Pose2d(13.474446, 4.745482, Rotation2d.fromDegrees(-120));
    public static final Pose2d Tag9Pose = new Pose2d(12.643358, 4.745482, Rotation2d.fromDegrees(-60));
    public static final Pose2d Tag10Pose = new Pose2d(12.227305999999999, 4.0259, new Rotation2d(0));
    public static final Pose2d Tag11Pose = new Pose2d(12.643358, 3.3063179999999996, Rotation2d.fromDegrees(60));

    public static final Pose2d Tag17Pose = new Pose2d(4.073905999999999, 3.3063179999999996,
            Rotation2d.fromDegrees(60));
    public static final Pose2d Tag18Pose = new Pose2d(3.6576, 4.0259, new Rotation2d(0));
    public static final Pose2d Tag19Pose = new Pose2d(4.073905999999999, 4.745482, Rotation2d.fromDegrees(-60));
    public static final Pose2d Tag20Pose = new Pose2d(4.904739999999999, 4.745482, Rotation2d.fromDegrees(-120));
    public static final Pose2d Tag21Pose = new Pose2d(5.321046, 4.0259, Rotation2d.fromDegrees(180));
    public static final Pose2d Tag22Pose = new Pose2d(4.904739999999999, 3.3063179999999996,
            Rotation2d.fromDegrees(120));

    // Source
    public static final Pose2d Tag1Pose = new Pose2d(16.697198, 0.65532, Rotation2d.fromDegrees(-52));
    public static final Pose2d Tag2Pose = new Pose2d(16.697198, 7.3964799999999995, Rotation2d.fromDegrees(52));
    public static final Pose2d Tag12Pose = new Pose2d(0.851154, 0.65532, Rotation2d.fromDegrees(-127));
    public static final Pose2d Tag13Pose = new Pose2d(0.851154, 7.3964799999999995, Rotation2d.fromDegrees(127));

}


package frc.robot.RobotControl;


import com.ma5951.utils.RobotControl.GenericSuperStracture;

import frc.robot.GlobalConstants;
import frc.robot.RobotContainer;
import frc.robot.Subsystem.Gripper.Gripper;
import frc.robot.Subsystem.Intake.IntakeRoller.IntakeRoller;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;

public class SuperStructure extends GenericSuperStracture implements GlobalConstants{

    public static IntakeRoller intakeRoller;
    public static Gripper gripper;
    private static Field.ScoringLevel scoringLevel;

    public SuperStructure() {
        super(() -> PoseEstimator.getInstance().getEstimatedRobotPose(), () -> SwerveSubsystem.getInstance().getVelocityVector());
        intakeRoller = RobotContainer.intakeRoller;
        gripper = RobotContainer.gripper;

        scoringLevel = Field.ScoringLevel.L3;
    }

    public static boolean hasGamePiece() {
        return intakeRoller.hasCoral() || gripper.hasCoral();
    }

    public static void setScoringLevel(Field.ScoringLevel ScoringLevel) {
        scoringLevel = ScoringLevel;
    }

    public static Field.ScoringLevel getScoringLevel() {
        return scoringLevel;
    }

    public void update() {
    }


}

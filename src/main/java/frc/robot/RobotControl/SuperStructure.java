
package frc.robot.RobotControl;


import com.ma5951.utils.RobotControl.GenericSuperStracture;

import frc.robot.RobotContainer;
import frc.robot.Subsystem.Gripper.Gripper;
import frc.robot.Subsystem.Intake.IntakeRoller.IntakeRoller;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;

public class SuperStructure extends GenericSuperStracture{

    public static IntakeRoller intakeRoller;
    public static Gripper gripper;

    public SuperStructure() {
        super(() -> PoseEstimator.getInstance().getEstimatedRobotPose(), () -> SwerveSubsystem.getInstance().getVelocityVector());
        intakeRoller = RobotContainer.intakeRoller;
        gripper = RobotContainer.gripper;
    }

    public static boolean hasGamePiece() {
        return intakeRoller.hasCoral() || gripper.hasCoral();
    }

    public void update() {

    }


}


package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Subsystem.Gripper.IOs.GripperIOSim;
import frc.robot.Subsystem.Intake.IntakeRoller.IOs.IntakeRollerIOSim;
import frc.robot.Subsystem.Swerve.SwerveConstants;

public class GamePieceSimulator {

    public enum CoralPose {
        NONE,
        GRIPPER,
        INTAKE
    }

    public static CoralPose coralPose = CoralPose.NONE;

    public static void updateSim() {

        if (RobotContainer.currentRobotState == RobotConstants.INTAKE
                && RobotContainer.intakeRoller.getVelocity() > 100) {
            IntakeRollerIOSim.intakeSim.startIntake();
        } else {
            IntakeRollerIOSim.intakeSim.stopIntake();
        }

        if (RobotContainer.intakeRoller.hasCoral()) {
            coralPose = CoralPose.INTAKE;
        }

        if (RobotContainer.currentRobotState == RobotConstants.HANDOFF
                && RobotContainer.intakeRoller.getVelocity() > 100 && RobotContainer.gripper.getVelocity() > 100
                && RobotContainer.intakeRoller.hasCoral()) {
            IntakeRollerIOSim.intakeSim.obtainGamePieceFromIntake();
            GripperIOSim.setHasCoral(true);
            coralPose = CoralPose.GRIPPER;
        } else {

        }

        if (RobotContainer.currentRobotState == RobotConstants.SCORING && RobotContainer.gripper.getVelocity() < -50
                && RobotContainer.gripper.hasCoral()) {
            SimulatedArena.getInstance()
                    .addGamePieceProjectile(new ReefscapeCoralOnFly(
                            // Obtain robot position from drive simulation
                            SwerveConstants.SWERVE_DRIVE_SIMULATION.getSimulatedDriveTrainPose().getTranslation(),
                            // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
                            new Translation2d(0.35, 0),
                            // Obtain robot speed from drive simulation
                            SwerveConstants.SWERVE_DRIVE_SIMULATION.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                            // Obtain robot facing from drive simulation
                            SwerveConstants.SWERVE_DRIVE_SIMULATION.getSimulatedDriveTrainPose().getRotation(),
                            // The height at which the coral is ejected
                            Meters.of(1.28),
                            // The initial speed of the coral
                            MetersPerSecond.of(2),
                            // The coral is ejected at a 35-degree slope
                            Degrees.of(-35)));
            GripperIOSim.setHasCoral(false);
            coralPose = CoralPose.NONE;

        }

    }

}

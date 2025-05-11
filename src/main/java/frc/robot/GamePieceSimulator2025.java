
package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Random;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

import com.ma5951.utils.Logger.MALog;
import com.ma5951.utils.RobotControl.Simulation.GamePieceSimulator;
import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Subsystem.Arm.ArmConstants;
import frc.robot.Subsystem.Elevator.ElevatorConstants;
import frc.robot.Subsystem.Gripper.IOs.GripperIOSim;
import frc.robot.Subsystem.Intake.IntakeArm.IntakeArmConstants;
import frc.robot.Subsystem.Intake.IntakeRoller.IOs.IntakeRollerIOSim;
import frc.robot.Subsystem.Swerve.SwerveConstants;

public class GamePieceSimulator2025 implements GamePieceSimulator {

    public enum CoralPose {
        NONE,
        GRIPPER,
        INTAKE
    }

    private static CoralPose coralPose;
    private static Pose3d coralPose3d;

    private static IntakeSimulation intakeSim;

    private int numOfCorals = 0;
    private Timer timeSinceLastCoral = new Timer();
    private Random random = new Random();

    public GamePieceSimulator2025() {
        intakeSim = IntakeSimulation.OverTheBumperIntake(
                "Coral",
                SwerveConstants.SWERVE_DRIVE_SIMULATION,
                Meters.of(0.44),
                Meters.of(0.20),
                IntakeSide.FRONT,
                1);

        coralPose = CoralPose.NONE;

        timeSinceLastCoral.start();

    }

    public void updateSim() {
        dropCorals();

        if (RobotContainer.currentRobotState == RobotConstants.INTAKE
                && RobotContainer.intakeRoller.getVelocity() > 100) {
            intakeSim.startIntake();
        } else {
            intakeSim.stopIntake();
        }

        if (intakeSim.getGamePiecesAmount() > 0) {
            IntakeRollerIOSim.setHasCoral(true);
            coralPose = CoralPose.INTAKE;
        } else {
            IntakeRollerIOSim.setHasCoral(false);
        }

        if (RobotContainer.currentRobotState == RobotConstants.HANDOFF
                && RobotContainer.intakeRoller.getVelocity() < -100 && RobotContainer.gripper.getVelocity() > 100
                && RobotContainer.intakeRoller.hasCoral()
                && RobotContainer.elevator.getHight() < 1.065) {
            intakeSim.obtainGamePieceFromIntake();
            GripperIOSim.setHasCoral(true);
            coralPose = CoralPose.GRIPPER;
        } else {

        }

        if (RobotContainer.currentRobotState == RobotConstants.SCORING && RobotContainer.gripper.getVelocity() < -50
                && RobotContainer.gripper.hasCoral()) {
            scoreBracnh();
            GripperIOSim.setHasCoral(false);
            coralPose = CoralPose.NONE;
        }

        MALog.log("Simulation/Coral Pose Enum", coralPose.name());
        coralPose3d = new Pose3d();

        if (coralPose == CoralPose.INTAKE) {
            displayCoralInIntake();
        } else if (coralPose == CoralPose.GRIPPER) {
            displayCoralInGripper();
        } else {
            clearCoralDisplay();
        }

    }

    private static void scoreBracnh() {
        SimulatedArena.getInstance()
                .addGamePieceProjectile(new ReefscapeCoralOnFly(
                        // Obtain robot position from drive simulation
                        SwerveConstants.SWERVE_DRIVE_SIMULATION.getSimulatedDriveTrainPose().getTranslation(),
                        // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
                        new Translation2d(0.67 * Math.sin(ConvUtil.DegreesToRadians(RobotContainer.arm.getPosition())),
                                -0.125),
                        // Obtain robot speed from drive simulation
                        SwerveConstants.SWERVE_DRIVE_SIMULATION.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                        // Obtain robot facing from drive simulation
                        Rotation2d.fromDegrees(SwerveConstants.SWERVE_DRIVE_SIMULATION.getSimulatedDriveTrainPose()
                                .getRotation().getDegrees() + 90),
                        // The height at which the coral is ejected
                        Meters.of((RobotContainer.elevator.getHight() + ElevatorConstants.SIM_ELEVATOR_OFFSET.getZ())
                                - (Math.cos(ConvUtil.DegreesToRadians(RobotContainer.arm.getPosition())) * 0.49)
                                + 0.03),
                        // The initial speed of the coral
                        MetersPerSecond.of(2),
                        // The coral is ejected at a 35-degree slope
                        Degrees.of(-(180 - RobotContainer.arm.getPosition()))));
    }

    private static void displayCoralInIntake() {
        coralPose3d = coralPose3d.transformBy(
                new Transform3d(new Translation3d(0.1875 + 0.4135, 0, (Inches.of(4.5).in(Meters) / 2) + 0.035),
                        new Rotation3d(0, 0, ConvUtil.DegreesToRadians(90))));

        coralPose3d = coralPose3d.rotateAround(
                IntakeArmConstants.SIM_INTAKE_OFFSET.getTranslation(),
                new Rotation3d(
                        0,
                        (ConvUtil.DegreesToRadians(-RobotContainer.intakeArm.getPosition())),
                        0));

        coralPose3d = coralPose3d.rotateBy(new Rotation3d(
                Degrees.of(0),
                Degrees.of(0),
                SwerveConstants.SWERVE_DRIVE_SIMULATION.getSimulatedDriveTrainPose().getRotation().getMeasure()));

        coralPose3d = new Pose3d(
                coralPose3d.getMeasureX()
                        .plus(SwerveConstants.SWERVE_DRIVE_SIMULATION.getSimulatedDriveTrainPose().getMeasureX()),
                coralPose3d.getMeasureY()
                        .plus(SwerveConstants.SWERVE_DRIVE_SIMULATION.getSimulatedDriveTrainPose().getMeasureY()),
                coralPose3d.getMeasureZ(),
                coralPose3d.getRotation());

        MALog.log("Simulation/Coral Pose", new Pose3d[] { coralPose3d });
    }

    private static void displayCoralInGripper() {
        coralPose3d = new Pose3d(new Translation3d(0.125, 0, RobotContainer.elevator.getHight() - 0.49),
                new Rotation3d(0, 0, ConvUtil.DegreesToRadians(90)));

        coralPose3d = coralPose3d.rotateAround(
                new Translation3d(ArmConstants.SIM_ARM_OFFSET.getTranslation().getX(),
                        ArmConstants.SIM_ARM_OFFSET.getTranslation().getY(),
                        RobotContainer.elevator.getHight()
                                + ElevatorConstants.SIM_ELEVATOR_OFFSET.getTranslation().getZ()),
                new Rotation3d(
                        (ConvUtil.DegreesToRadians(RobotContainer.arm.getPosition())),
                        0,
                        0));

        coralPose3d = coralPose3d.rotateBy(new Rotation3d(
                Degrees.of(0),
                Degrees.of(0),
                SwerveConstants.SWERVE_DRIVE_SIMULATION.getSimulatedDriveTrainPose().getRotation().getMeasure()));

        coralPose3d = new Pose3d(
                coralPose3d.getMeasureX()
                        .plus(SwerveConstants.SWERVE_DRIVE_SIMULATION.getSimulatedDriveTrainPose().getMeasureX()),
                coralPose3d.getMeasureY()
                        .plus(SwerveConstants.SWERVE_DRIVE_SIMULATION.getSimulatedDriveTrainPose().getMeasureY()),
                coralPose3d.getMeasureZ(),
                coralPose3d.getRotation());

        MALog.log("Simulation/Coral Pose", new Pose3d[] { coralPose3d });
    }

    private static void clearCoralDisplay() {
        MALog.log("Simulation/Coral Pose", new Pose3d[] {});
    }

    private void dropCorals() {
        numOfCorals = 0;
        for (@SuppressWarnings("unused") GamePieceOnFieldSimulation gamePiece : SimulatedArena.getInstance().gamePiecesOnField())
            numOfCorals++;

        if (numOfCorals < 15 && timeSinceLastCoral.get() > 5) {
            timeSinceLastCoral.reset();
            timeSinceLastCoral.start();
            SimulatedArena.getInstance()
                    .addGamePieceProjectile(new ReefscapeCoralOnFly(
                            // Obtain robot position from drive simulation
                            new Translation2d(0.9, 0.9),
                            // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
                            new Translation2d(0, 0),
                            // Obtain robot speed from drive simulation
                            new ChassisSpeeds(0, 0, 0),
                            // Obtain robot facing from drive simulation
                            Rotation2d.fromDegrees(random.nextInt(0, 90)),
                            // The height at which the coral is ejected
                            Meters.of(1),
                            // The initial speed of the coral
                            MetersPerSecond.of(random.nextDouble(1.5, 4)),
                            // The coral is ejected at a 35-degree slope
                            Degrees.of(-55)));
        }
    }
}

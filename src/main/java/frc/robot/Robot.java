
package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Random;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import com.ma5951.utils.Logger.MALog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;
import frc.robot.Subsystem.Swerve.SwerveConstants;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  public static boolean isStartingPose = false;
  private int numOfCorals = 0;
  private Timer timeSinceLastCoral = new Timer();
  private Random random = new Random();

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    MALog.resetID();


  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    PoseEstimator.getInstance().update();
    m_robotContainer.updatePeriodic();


  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.updateDisablePeriodic();
  }

  @Override
  public void autonomousInit() {
    m_robotContainer.updateAutoInit();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
    timeSinceLastCoral.start();
    SimulatedArena.getInstance().addDriveTrainSimulation(SwerveConstants.SWERVE_DRIVE_SIMULATION);
    SimulatedArena.getInstance().clearGamePieces();
    SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(
    // We must specify a heading since the coral is a tube
    new Pose2d(2, 2, Rotation2d.fromDegrees(90))));
    SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(
    // We must specify a heading since the coral is a tube
    new Pose2d(1, 1, Rotation2d.fromDegrees(90))));
    SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(
    // We must specify a heading since the coral is a tube
    new Pose2d(3, 2, Rotation2d.fromDegrees(90))));
    SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(
    // We must specify a heading since the coral is a tube
    new Pose2d(5, 3, Rotation2d.fromDegrees(90))));

 

 
  }

  @Override
  public void simulationPeriodic() {
    SimulatedArena.getInstance().simulationPeriodic();
    MALog.log("Simulation Pose", SwerveConstants.SWERVE_DRIVE_SIMULATION.getSimulatedDriveTrainPose());
    MALog.log("FieldSimulation/Coral", 
    SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    GamePieceSimulator.updateSim();

    // numOfCorals = 0;
    // for (GamePieceOnFieldSimulation gamePiece : SimulatedArena.getInstance().gamePiecesOnField()) numOfCorals++ ;

    // if (numOfCorals < 4 && timeSinceLastCoral.get() > 3) {
    //   timeSinceLastCoral.reset();
    //   timeSinceLastCoral.start();
    //   SimulatedArena.getInstance()
    //                 .addGamePieceProjectile(new ReefscapeCoralOnFly(
    //                         // Obtain robot position from drive simulation
    //                         new Translation2d(0.9, 0.9),
    //                         // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
    //                         new Translation2d(0, 0),
    //                         // Obtain robot speed from drive simulation
    //                         new ChassisSpeeds(0, 0, 0),
    //                         // Obtain robot facing from drive simulation
    //                         Rotation2d.fromDegrees(random.nextInt(10 , 80)),
    //                         // The height at which the coral is ejected
    //                         Meters.of(1),
    //                         // The initial speed of the coral
    //                         MetersPerSecond.of(1.5),
    //                         // The coral is ejected at a 35-degree slope
    //                         Degrees.of(-35)));
    // }

  }

}

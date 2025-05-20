
package frc.robot;

import com.ma5951.utils.Logger.MALog;
import com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.ConstantsClasses.CameraConstants;
import com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.Systems.VisionSystem.CameraIO;
import com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.Systems.VisionSystem.PhotonSimIO;
import com.ma5951.utils.RobotControl.Utils.Camera.Cameras;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;
import frc.robot.Subsystem.Swerve.SwerveConstants;
import frc.robot.Subsystem.Vision.VisionConstants;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  public static boolean isStartingPose = false;

  private SuperStructure m_superStructure;
  private CameraIO cameraIO;
  

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    MALog.resetID();

    m_superStructure = new SuperStructure();

    cameraIO = new PhotonSimIO(
      new CameraConstants(
        "Test", Cameras.LL4, VisionConstants.ROBOT_TO_CAMERA, false)
    );

    
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    PoseEstimator.getInstance().update();
    m_robotContainer.robotPeriodic();


    cameraIO.updatePeriodic(SwerveConstants.SWERVE_DRIVE_SIMULATION.getSimulatedDriveTrainPose());
    System.out.println(cameraIO.getFiducialData()[0].id);
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
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
    RobotContainer.simulationInit(true);
  }

  @Override
  public void simulationPeriodic() {
    RobotContainer.simulationPeriodic();
  }

}

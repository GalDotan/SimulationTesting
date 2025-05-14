
package frc.robot;

import com.ma5951.utils.Logger.MALog;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystem.IntakeV2.AlgeConstants;
import frc.robot.Subsystem.IntakeV2.IntakeAlge;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  public static boolean isStartingPose = false;

  private SuperStructure m_superStructure;
  

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    MALog.resetID();

    m_superStructure = new SuperStructure();

    
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    PoseEstimator.getInstance().update();
    m_robotContainer.robotPeriodic();



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

    AlgeConstants.ALGE_INTAKE.SENSORS[0].set(true);

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

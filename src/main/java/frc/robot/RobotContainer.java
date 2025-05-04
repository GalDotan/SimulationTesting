
package frc.robot;


import com.ma5951.utils.RobotControl.DeafultRobotContainer;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystem.Arm.Arm;
import frc.robot.Subsystem.Arm.ArmConstants;
import frc.robot.Subsystem.Arm.IOs.ArmIO;
import frc.robot.Subsystem.Arm.IOs.ArmIOSim;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;
import frc.robot.Subsystem.Swerve.SwerveAutoFollower;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;
import frc.robot.Subsystem.Vision.Vision;
import frc.robot.commands.Arm.ArmDeafultCommand;
import frc.robot.commands.Swerve.TeleopSwerveController;

public class RobotContainer extends DeafultRobotContainer{

  public static Arm arm;

  public static int x = 0;

  public RobotContainer() {
    super(
      PortMap.Controllers.driveID, 
      PortMap.Controllers.operatorID, 
      PortMap.Controllers.driveRumbleID,
      PortMap.Controllers.operatorRumbleID);
    SwerveSubsystem.getInstance();
    Vision.getInstance();
    PoseEstimator.getInstance();
    SwerveAutoFollower.getInstance();
    arm = Arm.getInstance();
    
    configureBindings();
    setUpAutoCommands();
    CommandScheduler.getInstance().setDefaultCommand(SwerveSubsystem.getInstance(), new TeleopSwerveController(driverController));
    CommandScheduler.getInstance().setDefaultCommand(arm, new ArmDeafultCommand());
  }

  public void setUpAutoCommands() {
    //setAutoOptions(null);
  }

  private void configureBindings() {

    //Update Offset
    new Trigger(() -> driverController.getYButton()).onTrue(new InstantCommand(() -> TeleopSwerveController.driveController.updateDriveHeading()));

    //Manuel Vision Update

    new Trigger(() -> arm.getTargetState() == ArmConstants.INTAKE && ArmIOSim.intakeSim.getGamePiecesAmount() > 0).onTrue(new InstantCommand(() -> arm.setTargetState(ArmConstants.HOLD)));
    new Trigger(() -> arm.getTargetState() == ArmConstants.HOLD && arm.getPosition() > 90 && ArmIOSim.intakeSim.getGamePiecesAmount() > 0 && (driverController.getLeftBumperButton() || driverController.getRightBumperButton()))
    .onTrue(new InstantCommand(() -> ArmIOSim.intakeSim.obtainGamePieceFromIntake())
    .alongWith(new InstantCommand(() -> x = 1))
    .alongWith(new InstantCommand(() -> arm.setTargetState(ArmConstants.SCORING))));


    new Trigger(() -> arm.getTargetState() == ArmConstants.SCORING && ArmIOSim.intakeSim.getGamePiecesAmount() < 1).onTrue(new InstantCommand(() -> arm.setTargetState(ArmConstants.IDLE)));
    new Trigger(() -> driverController.getRightBumperButton() && arm.getPosition() > 119).onTrue(new InstantCommand(() -> arm.setTargetState(ArmConstants.INTAKE)));
    //new Trigger(() -> driverController.getCrossButton()).onTrue(new InstantCommand(() -> arm.setTargetState(ArmConstants.HOLD)));
    new Trigger(() -> driverController.getLeftBumperButton() && arm.getTargetState() == ArmConstants.INTAKE).onTrue(new InstantCommand(() -> arm.setTargetState(ArmConstants.IDLE)));

    



  }

}
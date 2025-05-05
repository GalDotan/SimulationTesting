
package frc.robot;

import com.ma5951.utils.RobotControl.DeafultRobotContainer;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystem.Arm.Arm;
import frc.robot.Subsystem.Arm.ArmConstants;
import frc.robot.Subsystem.Gripper.Gripper;
import frc.robot.Subsystem.Gripper.GripperConstants;
import frc.robot.Subsystem.Intake.IntakeArm.IntakeArm;
import frc.robot.Subsystem.Intake.IntakeArm.IntakeArmConstants;
import frc.robot.Subsystem.Intake.IntakeRoller.IntakeRoller;
import frc.robot.Subsystem.Intake.IntakeRoller.IntakeRollerConstants;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;
import frc.robot.Subsystem.Swerve.SwerveAutoFollower;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;
import frc.robot.Subsystem.Vision.Vision;
import frc.robot.commands.Arm.ArmDeafultCommand;
import frc.robot.commands.Gripper.GripperDeafultCommand;
import frc.robot.commands.Intake.IntakeArmDeafultCommand;
import frc.robot.commands.Intake.IntakeRollerDeafultCommand;
import frc.robot.commands.Swerve.TeleopSwerveController;

public class RobotContainer extends DeafultRobotContainer {

  public static Arm arm;
  public static IntakeArm intakeArm;
  public static IntakeRoller intakeRoller;
  public static Gripper gripper;

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
    intakeArm = IntakeArm.getInstance();
    intakeRoller = IntakeRoller.getInstance();
    gripper = Gripper.getInstance();
    new SuperStructure();
    new GamePieceSimulator();

    configureBindings();

    CommandScheduler.getInstance().setDefaultCommand(SwerveSubsystem.getInstance(),
        new TeleopSwerveController(driverController));
    CommandScheduler.getInstance().setDefaultCommand(intakeArm, new IntakeArmDeafultCommand());
    CommandScheduler.getInstance().setDefaultCommand(intakeRoller, new IntakeRollerDeafultCommand());
    CommandScheduler.getInstance().setDefaultCommand(arm, new ArmDeafultCommand());
    CommandScheduler.getInstance().setDefaultCommand(gripper, new GripperDeafultCommand());
  }

  public static void setIDLE() {
    setCurrentState(RobotConstants.IDLE);
    arm.setTargetState(ArmConstants.IDLE);
    intakeArm.setTargetState(IntakeArmConstants.IDLE);
    intakeRoller.setTargetState(IntakeRollerConstants.IDLE);
    gripper.setTargetState(GripperConstants.IDLE);
  }

  public static void setINTAKE() {
    setCurrentState(RobotConstants.INTAKE);
    arm.setTargetState(ArmConstants.INTAKE);
    intakeArm.setTargetState(IntakeArmConstants.CORAL_INTAKE);
    intakeRoller.setTargetState(IntakeRollerConstants.CORAL_INTAKE);
    gripper.setTargetState(GripperConstants.IDLE);
  }

  public static void setHANDOFF() {
    setCurrentState(RobotConstants.HANDOFF);
    arm.setTargetState(ArmConstants.HANDOFF);
    intakeArm.setTargetState(IntakeArmConstants.HANDOFF);
    intakeRoller.setTargetState(IntakeRollerConstants.HANDOFF);
    gripper.setTargetState(GripperConstants.HANDOFF);
  }

  public static void setHOLD() {
    setCurrentState(RobotConstants.HOLD);
    arm.setTargetState(ArmConstants.HOLD);
    intakeArm.setTargetState(IntakeArmConstants.IDLE);
    intakeRoller.setTargetState(IntakeRollerConstants.IDLE);
    gripper.setTargetState(GripperConstants.HOLD);
  }

  private void configureBindings() {

    new Trigger(() -> (driverController.getR1Button() || driverController.getL1Button())
        && !SuperStructure.hasGamePiece()).onTrue(new InstantCommand(() -> setINTAKE()));

    new Trigger(() -> currentRobotState == RobotConstants.INTAKE
        && SuperStructure.hasGamePiece()).onTrue(new InstantCommand(() -> setHANDOFF()));

    new Trigger(() -> currentRobotState == RobotConstants.HANDOFF
        && gripper.hasCoral() && !intakeRoller.hasCoral()).onTrue(new InstantCommand(() -> setHOLD()));

    new Trigger(() -> driverController.getTouchpadButton()).onTrue(
      new InstantCommand(() -> setIDLE()));

  }

}
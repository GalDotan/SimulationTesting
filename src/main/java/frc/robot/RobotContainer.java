
package frc.robot;

import com.ma5951.utils.RobotControl.DeafultRobotContainer;
import com.ma5951.utils.RobotControl.StatesTypes.StatesConstants;
import com.ma5951.utils.RobotControlAdv.MARobotContainer;
import com.ma5951.utils.RobotControlAdv.TriggerManeger;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.RobotControl.Field.ScoringLevel;
import frc.robot.Subsystem.Arm.Arm;
import frc.robot.Subsystem.Arm.ArmConstants;
import frc.robot.Subsystem.Elevator.Elevator;
import frc.robot.Subsystem.Elevator.ElevatorConstants;
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
import frc.robot.commands.Elevator.ElevatorDeafultCommand;
import frc.robot.commands.Gripper.GripperDeafultCommand;
import frc.robot.commands.Intake.IntakeArmDeafultCommand;
import frc.robot.commands.Intake.IntakeRollerDeafultCommand;
import frc.robot.commands.Swerve.TeleopSwerveController;


public class RobotContainer extends MARobotContainer {

  public static Arm arm;
  public static IntakeArm intakeArm;
  public static IntakeRoller intakeRoller;
  public static Gripper gripper;
  public static Elevator elevator;
  public static SwerveSubsystem swerveSubsystem;
  public static Vision vision;
  public static PoseEstimator poseEstimator;
  public static SwerveAutoFollower swerveAutoFollower;

  public RobotContainer() {
    super();



    swerveSubsystem = SwerveSubsystem.getInstance();
    vision = Vision.getInstance();
    poseEstimator = PoseEstimator.getInstance();
    swerveAutoFollower = SwerveAutoFollower.getInstance();
    arm = Arm.getInstance();
    intakeArm = IntakeArm.getInstance();
    intakeRoller = IntakeRoller.getInstance();
    gripper = Gripper.getInstance();
    elevator = Elevator.getInstance();
    
    SuperStructure superStructure = new SuperStructure();
    GamePieceSimulator gamePieceSimulator = new GamePieceSimulator();

    configureBindings();

    CommandScheduler.getInstance().setDefaultCommand(SwerveSubsystem.getInstance(),
        new TeleopSwerveController(driverController));
    CommandScheduler.getInstance().setDefaultCommand(intakeArm, new IntakeArmDeafultCommand());
    CommandScheduler.getInstance().setDefaultCommand(intakeRoller, new IntakeRollerDeafultCommand());
    CommandScheduler.getInstance().setDefaultCommand(arm, new ArmDeafultCommand());
    CommandScheduler.getInstance().setDefaultCommand(gripper, new GripperDeafultCommand());
    CommandScheduler.getInstance().setDefaultCommand(elevator, new ElevatorDeafultCommand());
  }

  public static void setIDLE() {
    setCurrentState(RobotConstants.IDLE);
    arm.setTargetState(ArmConstants.IDLE);
    intakeArm.setTargetState(IntakeArmConstants.IDLE);
    intakeRoller.setTargetState(IntakeRollerConstants.IDLE);
    gripper.setTargetState(GripperConstants.IDLE);
    elevator.setTargetState(ElevatorConstants.IDLE);
  }

  public static void setINTAKE() {
    setCurrentState(RobotConstants.INTAKE);
    arm.setTargetState(ArmConstants.INTAKE);
    intakeArm.setTargetState(IntakeArmConstants.CORAL_INTAKE);
    intakeRoller.setTargetState(IntakeRollerConstants.CORAL_INTAKE);
    gripper.setTargetState(GripperConstants.IDLE);
    elevator.setTargetState(ElevatorConstants.INTAKE);
  }

  public static void setHANDOFF() {
    setCurrentState(RobotConstants.HANDOFF);
    arm.setTargetState(ArmConstants.HANDOFF);
    intakeArm.setTargetState(IntakeArmConstants.HANDOFF);
    intakeRoller.setTargetState(IntakeRollerConstants.HANDOFF);
    gripper.setTargetState(GripperConstants.HANDOFF);
    elevator.setTargetState(ElevatorConstants.HANDOFF);
  }

  public static void setHOLD() {
    setCurrentState(RobotConstants.HOLD);
    arm.setTargetState(ArmConstants.HOLD);
    intakeArm.setTargetState(IntakeArmConstants.IDLE);
    intakeRoller.setTargetState(IntakeRollerConstants.IDLE);
    gripper.setTargetState(GripperConstants.HOLD);
    elevator.setTargetState(ElevatorConstants.HOLD);
  }

  public static void setPRE_SCORING() {
    setCurrentState(RobotConstants.PRE_SCORING);
    arm.setTargetState(ArmConstants.PRE_SCORING);
    intakeArm.setTargetState(IntakeArmConstants.IDLE);
    intakeRoller.setTargetState(IntakeRollerConstants.IDLE);
    gripper.setTargetState(GripperConstants.IDLE);
    elevator.setTargetState(ElevatorConstants.PRE_SCORING);
  }

  public static void setSCORING() {
    setCurrentState(RobotConstants.SCORING);
    arm.setTargetState(ArmConstants.SCORING);
    intakeArm.setTargetState(IntakeArmConstants.IDLE);
    intakeRoller.setTargetState(IntakeRollerConstants.IDLE);
    gripper.setTargetState(GripperConstants.SCORING);
    elevator.setTargetState(ElevatorConstants.SCORING);
  }

  private void configureBindings() {

    new Trigger(() -> driverController.getBackButtonPressed())
        .onTrue(new InstantCommand(() -> TeleopSwerveController.driveController.updateDriveHeading()));

    new Trigger(() -> (driverController.getLeftBumperButton() || driverController.getRightBumperButton())
        && !SuperStructure.hasGamePiece()).onTrue(new InstantCommand(() -> setINTAKE()));

    new Trigger(() -> currentRobotState == RobotConstants.INTAKE
        && SuperStructure.hasGamePiece()).onTrue(new InstantCommand(() -> setHANDOFF()));

    new Trigger(() -> currentRobotState == RobotConstants.HANDOFF
        && gripper.hasCoral() && !intakeRoller.hasCoral()).onTrue(new InstantCommand(() -> setHOLD()));

       

    new Trigger(() -> driverController.getStartButton()).onTrue(
        new InstantCommand(() -> setIDLE()));

    new Trigger(() -> (driverController.getRightBumperButton())
        && gripper.hasCoral()).onTrue(new InstantCommand(() -> setPRE_SCORING()));

    new Trigger(() -> (driverController.getLeftBumperButton())
        && gripper.hasCoral()).onTrue(new InstantCommand(() -> setSCORING()));

        new Trigger(() -> driverController.getYButton())
        .onTrue(new InstantCommand(() -> SuperStructure.setScoringLevel(ScoringLevel.L4)));

        new Trigger(() -> driverController.getXButton())
        .onTrue(new InstantCommand(() -> SuperStructure.setScoringLevel(ScoringLevel.L3)));

        new Trigger(() -> driverController.getBButton())
        .onTrue(new InstantCommand(() -> SuperStructure.setScoringLevel(ScoringLevel.L2)));


  }

}

package frc.robot;

import com.ma5951.utils.RobotControl.Control.MARobotContainer;
import com.ma5951.utils.RobotControl.Controllers.MAXboxController;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.RobotControl.Field.ScoringLevel;
import frc.robot.Subsystem.Arm.Arm;
import frc.robot.Subsystem.Elevator.Elevator;
import frc.robot.Subsystem.Gripper.Gripper;
import frc.robot.Subsystem.Intake.IntakeArm.IntakeArm;
import frc.robot.Subsystem.Intake.IntakeRoller.IntakeRoller;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;
import frc.robot.Subsystem.Swerve.SwerveAutoFollower;
import frc.robot.Subsystem.Swerve.SwerveConstants;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;
import frc.robot.Subsystem.Vision.Vision;
import frc.robot.commands.Arm.ArmCommand;
import frc.robot.commands.Elevator.ElevatorCommand;
import frc.robot.commands.Gripper.GripperCommand;
import frc.robot.commands.Intake.IntakeArmCommand;
import frc.robot.commands.Intake.IntakeRollerCommand;
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
    poseEstimator = PoseEstimator.getInstance();
    swerveAutoFollower = SwerveAutoFollower.getInstance();
    arm = Arm.getInstance();
    intakeArm = IntakeArm.getInstance();
    intakeRoller = IntakeRoller.getInstance();
    gripper = Gripper.getInstance();
    elevator = Elevator.getInstance();
    
    withDriverController(new MAXboxController(0));
    wihtAddDeafultCommand(new ArmCommand());
    wihtAddDeafultCommand(new IntakeRollerCommand());
    wihtAddDeafultCommand(new IntakeArmCommand());
    wihtAddDeafultCommand(new GripperCommand());
    wihtAddDeafultCommand(new ElevatorCommand());
    wihtAddDeafultCommand(swerveSubsystem, new TeleopSwerveController(driverController));

    

    
    withSimulation(
      true,
      SwerveConstants.SWERVE_DRIVE_SIMULATION,
      new GamePieceSimulator2025(),
      new String[] {"Coral"}
    );
 
    configureBindings();

    @SuppressWarnings("unused")
    SuperStructure superStructure = new SuperStructure();
    
  }

  private void configureBindings() {

    

    T(() -> driverController.getOptionsRight() , () -> TeleopSwerveController.driveController.updateDriveHeading());

    T(() -> (driverController.getL1() || driverController.getR1())
    && !SuperStructure.hasGamePiece() , RobotConstants.INTAKE);

    T(RobotConstants.INTAKE ,() -> SuperStructure.hasGamePiece() , RobotConstants.HANDOFF);

    T(RobotConstants.HANDOFF ,() -> gripper.hasCoral() && !intakeRoller.hasCoral() , RobotConstants.HOLD);
   
    T(() -> driverController.getOptionsLeft() , RobotConstants.IDLE);

    T(() -> driverController.getR1() && gripper.hasCoral(), RobotConstants.PRE_SCORING);
    
    T(() -> driverController.getL1() && gripper.hasCoral(), RobotConstants.SCORING);

    T(() -> driverController.getActionsUp(), () -> SuperStructure.setScoringLevel(ScoringLevel.L4));
    
    T(() -> driverController.getActionsDown(), () -> SuperStructure.setScoringLevel(ScoringLevel.L2));
    
    T(() -> driverController.getActionsLeft(), () -> SuperStructure.setScoringLevel(ScoringLevel.L3));

  }

}
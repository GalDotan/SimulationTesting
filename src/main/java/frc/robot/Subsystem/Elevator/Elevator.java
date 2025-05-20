
package frc.robot.Subsystem.Elevator;

import com.ma5951.utils.RobotControl.Subsystems.StateControlledSubsystem;

import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.Subsystem.Elevator.IOs.ElevatorIO;
import frc.robot.Subsystem.Elevator.IOs.ElevatorIOReal;
import frc.robot.Subsystem.Elevator.IOs.ElevatorIOSim;

public class Elevator extends StateControlledSubsystem {
  private static Elevator elevator;

  private ElevatorIO elevatorIO;

  private Elevator() {
    super("Elevator");
    elevatorIO = getElevatorIO();
  }

  public static ElevatorIO getElevatorIO() {
        if (Robot.isReal()) {
            return new ElevatorIOReal();
        }

        return new ElevatorIOSim();
    }

  public void resetPose(double hight) {
    elevatorIO.resetPosition(hight);
  }

  public double getHight() {
    return elevatorIO.getPosition();
  }

  public double getVelocity() {
    return elevatorIO.getVelocity();
  }

  public double getAppliedVolts() {
    return elevatorIO.getAppliedVolts();
  }

  public double getCurrent() {
    return elevatorIO.getCurrent();
  }

  public boolean atPoint() {
    return Math.abs(elevatorIO.getError()) <= ElevatorConstants.TOLORANCE;
  }

  public double getSetPoint() {
    return elevatorIO.getSetPoint();
  }

  public void setNutralMode(boolean isBrake) {
    elevatorIO.setNutralMode(isBrake);
  }

  public void setVoltage(double volt) {
    elevatorIO.setVoltage(volt);
  }

  public void setHight(double hight) {
    elevatorIO.setHight(hight);
  }

  public boolean HandOffCanMove() {
    return RobotContainer.currentRobotState == RobotConstants.HANDOFF && RobotContainer.intakeArm.AtPoint();
  }

  @Override
  public boolean canMove() {
    return (HandOffCanMove() || RobotContainer.currentRobotState != RobotConstants.HANDOFF)  ;
  }

  public static Elevator getInstance() {
    if (elevator == null) {
      elevator = new Elevator();
    }
    return elevator;
  }

  @Override
  public void periodic() {
    super.periodic();
    elevatorIO.updatePeriodic();
  }
}

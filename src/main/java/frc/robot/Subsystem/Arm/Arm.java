package frc.robot.Subsystem.Arm;


import com.ma5951.utils.Logger.MALog;
import com.ma5951.utils.RobotControl.Subsystems.StateControlledSubsystem;
import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Subsystem.Arm.IOs.ArmIO;
import frc.robot.Subsystem.Arm.IOs.ArmIOReal;
import frc.robot.Subsystem.Arm.IOs.ArmIOSim;
import frc.robot.Subsystem.Elevator.ElevatorConstants;

public class Arm extends StateControlledSubsystem {
    private static Arm arm;

    private ArmIO armIO;

    private Arm() {
        super("Arm");
        armIO = getArmIO();
        System.out.println("11111111111111111111");
    }

    public static ArmIO getArmIO() {
        if (Robot.isReal()) {
            return new ArmIOReal();
        } else {
            return new ArmIOSim();
        }
    }

    public void resetMotorPose(double pose) {
        armIO.resetPosition(pose);
    }

    public double getFeedForwardVoltage() {
        return Math.sin(ConvUtil.DegreesToRadians(getPosition())) * ArmConstants.FEED_FORWARD_VOLTAGE;
    }

    public double getAbsolutePosition() {
        return armIO.getAbsolutePosition();
    }

    public double getCurrent() {
        return armIO.getCurrent();
    }

    public double getPosition() {
        return armIO.getPosition();
    }

    public double getVelocity() {
        return armIO.getVelocity();
    }

    public double getAppliedVolts() {
        return armIO.getAppliedVolts();
    }

    public double getSetPoint() {
        return armIO.getSetPoint();
    }

    public void resetPose() {
        armIO.resetPosition(getAbsolutePosition());
    }

    public void setNeutralMode(boolean isBrake) {
        armIO.setNeutralMode(isBrake);
    }

    public void setVoltage(double volt) {
        armIO.setVoltage(volt);
    }

    public void setAngle(double angle) {
        armIO.setAngle(angle, getFeedForwardVoltage());
    }

    public boolean AtPoint() {
        return Math.abs(armIO.getPosition() - armIO.getSetPoint()) < ArmConstants.TOLERANCE;
    }

    public boolean HitIntakeCanMove() {
        double intersectHight = getPosition() > 90 && getPosition() < 270 ? 
        (RobotContainer.elevator.getHight() + ElevatorConstants.SIM_ELEVATOR_OFFSET.getZ()) - (Math.cos(ConvUtil.DegreesToRadians(getPosition())) * ArmConstants.INTAKE_COSTRAINT_RADIUS) : 
        (RobotContainer.elevator.getHight() + ElevatorConstants.SIM_ELEVATOR_OFFSET.getZ()) - (Math.cos(ConvUtil.DegreesToRadians(getPosition())) * ArmConstants.ENDEFFECTOR_RADIUS) ;
        
        MALog.log("/Subsystems/Arm/Hit Intake Hight", intersectHight);
        return intersectHight > 0.61 + 0.05;
    }

    @Override
    public boolean canMove() {
        return HitIntakeCanMove();
    }

    public static Arm getInstance() {
        if (arm == null) {
            arm = new Arm();
        }
        return arm;
    }

    @Override
    public void periodic() {
        super.periodic();
        armIO.updatePeriodic();

        MALog.log("/Subsystems/Arm/At Point", AtPoint());

        if (!Robot.isReal()) {
            MALog.log("/Subsystems/Arm/Arm Position", new Pose3d(
                new Translation3d(ArmConstants.SIM_ARM_OFFSET.getTranslation().getX(), ArmConstants.SIM_ARM_OFFSET.getTranslation().getY(), 
                RobotContainer.elevator.getHight() + ElevatorConstants.SIM_ELEVATOR_OFFSET.getTranslation().getZ()),
                new Rotation3d(ConvUtil.DegreesToRadians(getPosition()), 0, 0)));
            
        }

        
    }
}

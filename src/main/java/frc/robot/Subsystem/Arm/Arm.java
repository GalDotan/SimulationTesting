package frc.robot.Subsystem.Arm;


import com.ma5951.utils.Logger.MALog;
import com.ma5951.utils.RobotControl.Subsystems.StateControlledSubsystem;
import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.Robot;
import frc.robot.Subsystem.Arm.IOs.ArmIO;

public class Arm extends StateControlledSubsystem {
    private static Arm arm;

    private ArmIO armIO = ArmConstants.getArmIO();

    private Arm() {
        super(ArmConstants.SUBSYSTEM_STATES, "Arm");

        setTargetState(ArmConstants.IDLE);
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

    @Override
    public boolean canMove() {
        return true;
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


        if (!Robot.isReal()) {
            MALog.log("/Subsystems/Arm/Arm Position", new Pose3d(ArmConstants.SIM_ARM_OFFSET.getTranslation(),
                new Rotation3d(ConvUtil.DegreesToRadians(getPosition()), 0, 0)));
        }

        
    }
}

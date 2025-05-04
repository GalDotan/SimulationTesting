package frc.robot.Subsystem.Arm;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ma5951.utils.Logger.MALog;
import com.ma5951.utils.RobotControl.Subsystems.StateControlledSubsystem;
import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.RobotConstants;
import frc.robot.Subsystem.Arm.IOs.ArmIO;
import frc.robot.Subsystem.Arm.IOs.ArmIOSim;
import frc.robot.Subsystem.Swerve.SwerveConstants;

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

        MALog.log("/Subsystems/Arm/Intake Position", new Pose3d(ArmConstants.SIM_ARM_OFFSET.getTranslation(),
                new Rotation3d(0, ConvUtil.DegreesToRadians(-getPosition()), 0)));

        Pose3d coralPose = new Pose3d();
        coralPose = coralPose.transformBy(
                new Transform3d(new Translation3d(0.2175 + 0.4135, 0, Inches.of(4.5).in(Meters) / 2),
                        new Rotation3d(0, 0, ConvUtil.DegreesToRadians(90))));

        coralPose = coralPose.rotateAround(
                ArmConstants.SIM_ARM_OFFSET.getTranslation(),
                new Rotation3d(
                        0,
                        (ConvUtil.DegreesToRadians(-getPosition())),
                        0));

        coralPose = coralPose.rotateBy(new Rotation3d(
                Degree.of(0),
                Degree.of(0),
                SwerveConstants.SWERVE_DRIVE_SIMULATION.getSimulatedDriveTrainPose().getRotation().getMeasure()));

        coralPose = new Pose3d(
                coralPose.getMeasureX()
                        .plus(SwerveConstants.SWERVE_DRIVE_SIMULATION.getSimulatedDriveTrainPose().getMeasureX()),
                coralPose.getMeasureY()
                        .plus(SwerveConstants.SWERVE_DRIVE_SIMULATION.getSimulatedDriveTrainPose().getMeasureY()),
                coralPose.getMeasureZ(),
                coralPose.getRotation());

        MALog.log("/Subsystems/Arm/Coral Position", new Pose3d[] { coralPose });

        if (ArmIOSim.intakeSim.getGamePiecesAmount() > 0) {
            MALog.log("/Subsystems/Arm/Coral Position", new Pose3d[] { coralPose });
        } else {
            MALog.log("/Subsystems/Arm/Coral Position", new Pose3d[0]);
        }
    }
}

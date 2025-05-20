
package frc.robot.Subsystem.Intake.IntakeArm;

import com.ma5951.utils.Logger.MALog;
import com.ma5951.utils.RobotControl.Subsystems.StateControlledSubsystem;
import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.Robot;
import frc.robot.Subsystem.Intake.IntakeArm.IOs.IntakeArmIO;
import frc.robot.Subsystem.Intake.IntakeArm.IOs.IntakeArmIOReal;
import frc.robot.Subsystem.Intake.IntakeArm.IOs.IntakeArmIOSim;


public class IntakeArm extends StateControlledSubsystem{
    private static IntakeArm intake;

    private IntakeArmIO intakeIO ;

    private IntakeArm() {
        super("Intake Arm");
        intakeIO = getIntakeIO();

        intakeIO.setNeutralMode(true);
    }

    public static IntakeArmIO getIntakeIO() {
        if (Robot.isReal()) {
            return new IntakeArmIOReal();
        } else {
            return new IntakeArmIOSim();
        }
    }

    public double getCurrent() {
        return intakeIO.getCurrent();
    }

    public double getPosition() {
        return intakeIO.getPosition();
    }

    public double getVolts() {
        return intakeIO.getAppliedVolts();
    }

    public double getSetPoint() {
        return intakeIO.getSetPoint();
    }

    public double getError() {
        return intakeIO.getError();
    }

    public boolean AtPoint() {
        return Math.abs(intakeIO.getPosition() - intakeIO.getSetPoint()) < IntakeArmConstants.TOLERANCE;
    }

    public void setVoltage(double voltage) {
        intakeIO.setVoltage(voltage);
    }

    public double getFeedForwardVoltage() {
        return IntakeArmConstants.FEED_FORWARD_VOLTAGE * Math.cos(getPosition());
    }

    public void setArmAngle(double angle) {
        intakeIO.setAngle(angle, 0);
    }

    @Override
    public boolean canMove() {
        return true;
    }

    public static IntakeArm getInstance() {
        if (intake == null) {
            intake = new IntakeArm();
        }
        return intake;
    }
    
    @Override
    public void periodic() {
        super.periodic();
        intakeIO.updatePeriodic();

        MALog.log("/Subsystems/Intake/Intake Arm/At Point", AtPoint());

        
        if (!Robot.isReal()) {
            MALog.log("/Subsystems/Intake/Intake Arm/Intake Position", new Pose3d(IntakeArmConstants.SIM_INTAKE_OFFSET.getTranslation(),
                new Rotation3d(0, ConvUtil.DegreesToRadians(-getPosition()), 0)));
        }
    }


}

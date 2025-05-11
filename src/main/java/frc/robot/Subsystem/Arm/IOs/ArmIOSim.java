package frc.robot.Subsystem.Arm.IOs;

import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Subsystem.Arm.ArmConstants;

public class ArmIOSim extends ArmIOReal {

    private TalonFXSimState motorSimState;

    private SingleJointedArmSim armSim;

    public ArmIOSim() {
        super();

        motorSimState = armMotor.getSimState();

        armSim = new SingleJointedArmSim(
                DCMotor.getKrakenX60(1),
                armConfig.Feedback.SensorToMechanismRatio,
                0.11,
                0.5,
                ConvUtil.DegreesToRadians(ArmConstants.MIN_ANGLE),
                ConvUtil.DegreesToRadians(ArmConstants.MAX_ANGLE),
                false,
                ConvUtil.DegreesToRadians(180));

    }

    @Override
    public void updatePeriodic() {

        motorSimState.setSupplyVoltage(12);
        armSim.setInputVoltage(motorSimState.getMotorVoltage());
        armSim.update(0.02);

        motorSimState.setRawRotorPosition(
                ConvUtil.RadiansToRotations(armSim.getAngleRads()) * armConfig.Feedback.SensorToMechanismRatio);
        motorSimState.setRotorVelocity(
                (armSim.getVelocityRadPerSec() * 0.1591549430919) * armConfig.Feedback.SensorToMechanismRatio);

        super.updatePeriodic();

    }
}

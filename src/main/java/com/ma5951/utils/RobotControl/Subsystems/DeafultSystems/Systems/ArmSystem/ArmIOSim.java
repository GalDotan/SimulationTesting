
package com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.Systems.ArmSystem;

import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.ConstantsClasses.ArmSystemConstants;
import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim extends ArmIOReal {

    private TalonFXSimState motorSimState;
    private SingleJointedArmSim armSim;

    public ArmIOSim(ArmSystemConstants systemConstants) {
        super(systemConstants);
        motorConfig.MotorOutput.Inverted = systemConstants.MOTORS[0].direction;

        motorSimState = systemConstants.MOTORS[0].talonFX.getSimState();

        armSim = new SingleJointedArmSim(
                DCMotor.getKrakenX60(1),
                motorConfig.Feedback.SensorToMechanismRatio,
                systemConstants.INERTIA,
                systemConstants.ARM_LENGTH,
                ConvUtil.DegreesToRadians(systemConstants.MIN_ANGLE),
                ConvUtil.DegreesToRadians(systemConstants.MAX_ANGLE),
                false,
                ConvUtil.DegreesToRadians(systemConstants.START_ANGLE));

    }

    @Override
    public void updatePeriodic() {
        motorSimState.setSupplyVoltage(12);
        armSim.setInputVoltage(motorSimState.getMotorVoltage());
        armSim.update(0.02);

        motorSimState.setRawRotorPosition(
                ConvUtil.RadiansToRotations(armSim.getAngleRads()) * motorConfig.Feedback.SensorToMechanismRatio);
        motorSimState.setRotorVelocity(
                (armSim.getVelocityRadPerSec() * 0.1591549430919) * motorConfig.Feedback.SensorToMechanismRatio);

        super.updatePeriodic();


    }

}

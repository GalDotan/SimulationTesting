
package com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.Systems.ArmSystem;

import com.ma5951.utils.ControlledMotors.Sim.TalonFXMotorSim;
import com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.ConstantsClasses.RollerSystemConstants;

import edu.wpi.first.math.system.plant.DCMotor;

public class ArmIOSim  extends ArmIOReal{

    private TalonFXMotorSim motorSim;

    public ArmIOSim(RollerSystemConstants systemConstants) {
        super(systemConstants);
        motorConfig.MotorOutput.Inverted = systemConstants.MOTORS[0].direction;
        motorSim = new TalonFXMotorSim(systemConstants.MOTORS[0].talonFX , motorConfig , DCMotor.getKrakenX60(1) , systemConstants.INERTIA , false);
    }

    @Override
    public void updatePeriodic() {
        super.updatePeriodic();
        motorSim.updateSim();
    }
    
}

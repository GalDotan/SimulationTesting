
package com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.Systems.RollerSystem;

import com.ma5951.utils.ControlledMotors.Sim.TalonFXMotorSim;
import com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.ConstantsClasses.RollerSystemConstants;

import edu.wpi.first.math.system.plant.DCMotor;

public class RollerIOSim  extends RollerIOReal{

    private TalonFXMotorSim motorSim;

    public RollerIOSim(RollerSystemConstants systemConstants) {
        super(systemConstants);
        motorSim = new TalonFXMotorSim(systemConstants.MOTORS[0].talonFX , motorConfig , DCMotor.getKrakenX60(1) , systemConstants.INERTIA , false);
    }

    @Override
    public void updatePeriodic() {
        super.updatePeriodic();
        motorSim.updateSim();
    }
    
}

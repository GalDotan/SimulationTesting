
package com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.Systems.PositionControlled;

import com.ma5951.utils.ControlledMotors.Sim.TalonFXMotorSim;
import com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.ConstantsClasses.PositionSystemConstants;

public class PositionControllerSim extends PositionControllerReal {

    private TalonFXMotorSim motorSim;

    public PositionControllerSim(PositionSystemConstants systemConstants) {
        super(systemConstants);
        motorConfig.MotorOutput.Inverted = systemConstants.MOTORS[0].direction;

        motorSim = new TalonFXMotorSim(systemConstants.MOTORS[0].talonFX, motorConfig,
                systemConstants.MOTORS[0].motorType.motorType,
                systemConstants.INERTIA, false);

    }

    @Override
    public void updatePeriodic() {
        super.updatePeriodic();
        motorSim.updateSim();
    }

}

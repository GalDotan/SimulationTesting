
package frc.robot.Subsystem.IntakeV2;

import com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.ConstantsClasses.RollerSystemConstants;
import com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.Systems.RollerSystem.RollerSubsystem;

public class IntakeAlge extends RollerSubsystem {
    private static IntakeAlge algeIntake;

    private IntakeAlge(String name, RollerSystemConstants systemConstants) {
        super(name, systemConstants);
    }

    public static IntakeAlge getInstance() {
        if (algeIntake == null) {
            algeIntake = new IntakeAlge("IntakeAloge", AlgeConstants.ALGE_INTAKE);
        }
        return algeIntake;
    }

    @Override
    public void periodic() {
        super.periodic();
        setVoltage(-12);
    }
}

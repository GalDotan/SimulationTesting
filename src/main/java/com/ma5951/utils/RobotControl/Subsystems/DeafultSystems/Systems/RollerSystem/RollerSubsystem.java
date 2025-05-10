
package com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.Systems.RollerSystem;

import com.ma5951.utils.RobotControl.Subsystems.StateControlledSubsystem;
import com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.ConstantsClasses.RollerSystemConstants;

public class RollerSubsystem extends StateControlledSubsystem{

    protected RollerSystemConstants systemConstants;

    public RollerSubsystem(String name,RollerSystemConstants systemConstants) {
        super(name);
        this.systemConstants = systemConstants;
    }

    


}

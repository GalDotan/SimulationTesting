
package com.ma5951.utils.RobotControl.StatesTypes;

import edu.wpi.first.wpilibj.DriverStation;

public class StatesConstants {
    public final static RobotOporationState TELEOP = new RobotOporationState("TELEOP");
    public final static RobotOporationState AUTO = new RobotOporationState("AUTO");
    public final static RobotOporationState TEST = new RobotOporationState("TEST");
    public final static SystemFunctionState AUTOMATIC = new SystemFunctionState("AUTOMATIC");
    public final static SystemFunctionState MANUEL = new SystemFunctionState("MANUEL");
    public final static GeneralSubsystemState IDLE = new GeneralSubsystemState("IDLE");


    public static RobotOporationState getRobotState() {
        if (DriverStation.isAutonomousEnabled()){
            return AUTO;
        } else if (DriverStation.isTeleopEnabled()) {
            return TELEOP;
        } else if (DriverStation.isTestEnabled()) {
            return TEST;
        } else {
            return null;
        }
    }
    
}

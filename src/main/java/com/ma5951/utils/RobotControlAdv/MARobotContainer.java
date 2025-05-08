
package com.ma5951.utils.RobotControlAdv;


import com.ma5951.utils.RobotControl.StatesTypes.StatesConstants;
import com.ma5951.utils.RobotControlAdv.Controllers.MAController;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;


public class MARobotContainer {

    public MAController driverController;
    public MAController oporatorController;

    public static TriggerManeger triggerManeger;


    public MARobotContainer() {
        triggerManeger = new TriggerManeger(
        () -> RobotContainer.currentRobotState,
        () -> StatesConstants.getRobotState());
    }

    public MARobotContainer withDriverController(MAController controller) {
        this.driverController = controller;
        return this;
    }

    public MARobotContainer withOporatorController(MAController controller) {
        this.driverController = controller;
        return this;
    }

    public MARobotContainer withDriverController(MAController controller, MAController oporatorController) {
        this.driverController = controller;
        this.oporatorController = oporatorController;
        return this;
    }

    public MARobotContainer wihtAddDeafultCommand(SystemDeafultCommand command) {
        CommandScheduler.getInstance().setDefaultCommand(command.subsystem, new DeafultCommandBuilder(command));
        return this;
    }

    public MARobotContainer stopDeafultCommand(SystemDeafultCommand command) {
        CommandScheduler.getInstance().removeDefaultCommand(command.subsystem);
        return this;
    }


}

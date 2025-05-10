
package com.ma5951.utils.RobotControl.Control;

import java.util.List;
import java.util.function.BooleanSupplier;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import com.ma5951.utils.Logger.MALog;
import com.ma5951.utils.RobotControl.Commands.DeafultCommandBuilder;
import com.ma5951.utils.RobotControl.Commands.SystemDeafultCommand;
import com.ma5951.utils.RobotControl.Controllers.MAController;
import com.ma5951.utils.RobotControl.Simulation.GamePieceSimulator;
import com.ma5951.utils.RobotControl.StatesTypes.RobotOporationState;
import com.ma5951.utils.RobotControl.StatesTypes.RobotStateMA;
import com.ma5951.utils.RobotControl.StatesTypes.StatesConstants;
import com.ma5951.utils.RobotControl.Utils.StatusSignalsRunner;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class MARobotContainer {

    public static MAController driverController;
    public static MAController oporatorController;

    private static TriggerManeger triggerManeger;

    public static RobotStateMA currentRobotState;
    public static RobotStateMA lastRobotState;

    private static boolean useSimulation = false;

    private static GamePieceSimulator gamePieceSimulator = null;
    private static SwerveDriveSimulation swerveDriveSimulation = null;

    private static List<String> gamePiecesList = null;

    public MARobotContainer() {
        triggerManeger = new TriggerManeger(
                () -> currentRobotState,
                () -> StatesConstants.getRobotState());

    }

    // Config
    @SuppressWarnings("static-access")
    public MARobotContainer withDriverController(MAController controller) {
        this.driverController = controller;
        return this;
    }

    @SuppressWarnings("static-access")
    public MARobotContainer withOporatorController(MAController controller) {
        this.oporatorController = controller;
        return this;
    }

    @SuppressWarnings("static-access")
    public MARobotContainer withDriverController(MAController controller, MAController oporatorController) {
        this.driverController = controller;
        this.oporatorController = oporatorController;
        return this;
    }

    public MARobotContainer withSimulation(boolean simulation, SwerveDriveSimulation swerveDriveSimulation , String... gamePieceType) {
        useSimulation = simulation;
        SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);
        for (String type : gamePieceType) {
            gamePiecesList.add(type);
        }
        return this;
    }

    @SuppressWarnings("static-access")
    public MARobotContainer withSimulation(boolean simulation, SwerveDriveSimulation swerveDriveSimulation,
            GamePieceSimulator gamePieceSimulator) {
        useSimulation = simulation;
        SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);
        this.gamePieceSimulator = gamePieceSimulator;
        this.swerveDriveSimulation = swerveDriveSimulation;
        return this;
    }

    // Deafult Command
    public MARobotContainer wihtAddDeafultCommand(SystemDeafultCommand command) {
        CommandScheduler.getInstance().setDefaultCommand(command.getSubsystem(), new DeafultCommandBuilder(command));
        return this;
    }

    public MARobotContainer stopDeafultCommand(SystemDeafultCommand command) {
        CommandScheduler.getInstance().removeDefaultCommand(command.getSubsystem());
        return this;
    }

    // Triggers
    public void T(RobotOporationState workInMode, RobotStateMA workInState, BooleanSupplier condition,
            Runnable action) {
        triggerManeger.add(workInMode, workInState, condition, action);
    }

    public void T(RobotStateMA workInState, BooleanSupplier condition, Runnable action) {
        triggerManeger.add(workInState, condition, action);
    }

    public void T(RobotOporationState workInMode, BooleanSupplier condition, Runnable action) {
        triggerManeger.add(workInMode, condition, action);
    }

    public void T(BooleanSupplier condition, Runnable action) {
        triggerManeger.add(condition, action);
    }

    // Robot States
    public void setRobotState(RobotStateMA robotState) {
        lastRobotState = currentRobotState;
        currentRobotState = robotState;
        robotState.setState();
    }

    public RobotStateMA getCurrentRobotState() {
        return currentRobotState;
    }

    public RobotStateMA getLastRobotState() {
        return lastRobotState;
    }

    // Periodics
    public void robotPeriodic() {
        StatusSignalsRunner.updateSignals();
        CommandScheduler.getInstance().run();
    } 

    public void simulationPeriodic() {
        if (useSimulation) {
            SimulatedArena.getInstance().simulationPeriodic();
            MALog.log("/Simulation/Simulation Pose", swerveDriveSimulation.getSimulatedDriveTrainPose());
            for (String type : gamePiecesList) {
                MALog.log("Simulation/GamePices/" + type, SimulatedArena.getInstance().getGamePiecesArrayByType(type));
            }
            if (gamePieceSimulator != null) {
                gamePieceSimulator.updateSim();
            }
        }
    }

    public void simulationInit(boolean autoGamePices) {
        if (useSimulation) {
            SimulatedArena.getInstance().clearGamePieces();
            if (autoGamePices) {
                SimulatedArena.getInstance().resetFieldForAuto();
            }
        }
    }

    public void simulationInit(boolean autoGamePices, Runnable toRun) {
        if (useSimulation) {
            SimulatedArena.getInstance().clearGamePieces();
            if (autoGamePices) {
                SimulatedArena.getInstance().resetFieldForAuto();
            }
            toRun.run();
        }
    }

}

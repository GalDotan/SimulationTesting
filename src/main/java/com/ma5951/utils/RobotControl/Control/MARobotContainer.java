
package com.ma5951.utils.RobotControl.Control;

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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MARobotContainer {

    public static MAController driverController;
    public static MAController oporatorController;

    private static TriggerManeger triggerManeger;

    public static RobotStateMA currentRobotState;
    public static RobotStateMA lastRobotState;

    private static boolean useSimulation = false;

    private static GamePieceSimulator gamePieceSimulator = null;
    private static SwerveDriveSimulation swerveDriveSimulation = null;

    private static String[] gamePiecesList ;

    public MARobotContainer() {
        triggerManeger = new TriggerManeger(
                () -> currentRobotState,
                () -> StatesConstants.getRobotState());

    }

    // Config
    public static void withDriverController(MAController controller) {
        driverController = controller;
    }

    public static void withOporatorController(MAController controller) {
        oporatorController = controller;
    }

    
    public static void withDriverController(MAController controller, MAController newOporatorController) {
        driverController = controller;
        oporatorController = newOporatorController;
    }

    public MARobotContainer withSimulation(boolean simulation, SwerveDriveSimulation swerveDriveSimulation,
    String[] gamePieceType) {
        useSimulation = simulation;
        SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);
        gamePiecesList = gamePieceType;
        return this;
    }

    public static void withSimulation(boolean simulation, SwerveDriveSimulation newSwerveDriveSimulation,
            GamePieceSimulator newGamePieceSimulator, String[] gamePieceType) {
        useSimulation = simulation;
        SimulatedArena.getInstance().addDriveTrainSimulation(newSwerveDriveSimulation);
        gamePieceSimulator = newGamePieceSimulator;
        swerveDriveSimulation = newSwerveDriveSimulation;
        gamePiecesList = gamePieceType;
    }

    // Deafult Command
    public MARobotContainer wihtAddDeafultCommand(SystemDeafultCommand command) {
        CommandScheduler.getInstance().setDefaultCommand(command.getSubsystem(), new DeafultCommandBuilder(command));
        return this;
    }

    public MARobotContainer wihtAddDeafultCommand(SubsystemBase system, Command command) {
        CommandScheduler.getInstance().setDefaultCommand(system, command);
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

    // State Triggers
    public void T(RobotOporationState workInMode, RobotStateMA workInState, BooleanSupplier condition,
            RobotStateMA robotState) {
        triggerManeger.add(workInMode, workInState, condition, () -> setRobotState(robotState));
    }

    public void T(RobotStateMA workInState, BooleanSupplier condition, RobotStateMA robotState) {
        triggerManeger.add(workInState, condition, () -> setRobotState(robotState));
    }

    public void T(RobotOporationState workInMode, BooleanSupplier condition, RobotStateMA robotState) {
        triggerManeger.add(workInMode, condition, () -> setRobotState(robotState));
    }

    public void T(BooleanSupplier condition, RobotStateMA robotState) {
        triggerManeger.add(condition, () -> setRobotState(robotState));
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

    // Chooser
    public Command getAutonomousCommand() {
        return null;
    }

    // Periodics
    public void robotPeriodic() {
        StatusSignalsRunner.updateSignals();
        CommandScheduler.getInstance().run();
    }

    public static void simulationPeriodic() {
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

    public static void simulationInit(boolean autoGamePices) {
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

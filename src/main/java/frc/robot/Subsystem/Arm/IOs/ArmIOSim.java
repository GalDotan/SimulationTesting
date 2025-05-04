package frc.robot.Subsystem.Arm.IOs;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ma5951.utils.ControlledMotors.Sim.TalonFXMotorSim;
import com.ma5951.utils.Logger.MALog;
import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.RobotContainer;
import frc.robot.Subsystem.Arm.Arm;
import frc.robot.Subsystem.Arm.ArmConstants;
import frc.robot.Subsystem.Swerve.SwerveConstants;

public class ArmIOSim extends ArmIOReal {

    private TalonFXSimState motorSimState;
    public static IntakeSimulation intakeSim;

    private SingleJointedArmSim armSim;

    public ArmIOSim() {
        super();

        motorSimState = armMotor.getSimState();

        armSim = new SingleJointedArmSim(
                DCMotor.getKrakenX60(1),
                armConfig.Feedback.SensorToMechanismRatio,
                0.09,
                0.5,
                ConvUtil.DegreesToRadians(ArmConstants.MIN_ANGLE),
                ConvUtil.DegreesToRadians(ArmConstants.MAX_ANGLE),
                false,
                ConvUtil.DegreesToRadians(ArmConstants.MAX_ANGLE));

        intakeSim = IntakeSimulation.OverTheBumperIntake(
                "Coral",
                SwerveConstants.SWERVE_DRIVE_SIMULATION,
                Meter.of(0.44),
                Meter.of(0.2625),
                IntakeSide.FRONT,
                1);
    }

    @Override
    public void updatePeriodic() {
        super.updatePeriodic();

        motorSimState.setSupplyVoltage(12);
        armSim.setInputVoltage(motorSimState.getMotorVoltage());
        armSim.update(0.02);

        motorSimState.setRawRotorPosition(
                ConvUtil.RadiansToRotations(armSim.getAngleRads()) * armConfig.Feedback.SensorToMechanismRatio);
        motorSimState.setRotorVelocity(
                (armSim.getVelocityRadPerSec() * 0.1591549430919) * armConfig.Feedback.SensorToMechanismRatio);

        if (Math.abs(getPosition()) < 5 && intakeSim.getGamePiecesAmount() < 1) {
            intakeSim.startIntake();
        } else {
            intakeSim.stopIntake();
        }

        if (Math.abs(getPosition()) < 5 && intakeSim.getGamePiecesAmount() < 1) {
            intakeSim.startIntake();
        } else {
            intakeSim.stopIntake();
        }

        if (intakeSim.getGamePiecesAmount() < 1 && RobotContainer.x == 1 && Math.abs(getPosition() - getSetPoint()) < 2) {

            SimulatedArena.getInstance()
                    .addGamePieceProjectile(new ReefscapeCoralOnFly(
                            // Obtain robot position from drive simulation
                            SwerveConstants.SWERVE_DRIVE_SIMULATION.getSimulatedDriveTrainPose().getTranslation(),
                            // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
                            new Translation2d(0.413, 0),
                            // Obtain robot speed from drive simulation
                            SwerveConstants.SWERVE_DRIVE_SIMULATION.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                            // Obtain robot facing from drive simulation
                            SwerveConstants.SWERVE_DRIVE_SIMULATION.getSimulatedDriveTrainPose().getRotation(),
                            // The height at which the coral is ejected
                            Meter.of(0.4),
                            // The initial speed of the coral
                            MetersPerSecond.of(2),
                            // The coral is ejected at a 35-degree slope
                            Degrees.of(35)));
            RobotContainer.x = 0;

        }

    }
}

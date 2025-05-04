package frc.robot.Subsystem.Intake.IntakeRoller.IOs;

import static edu.wpi.first.units.Units.Meter;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import com.ma5951.utils.ControlledMotors.Sim.TalonFXMotorSim;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Subsystem.Swerve.SwerveConstants;

public class IntakeRollerIOSim extends IntakeRollerIOReal {

    private TalonFXMotorSim motorSim;
    public static IntakeSimulation intakeSim;


    public IntakeRollerIOSim() {
        super();

        motorSim = new TalonFXMotorSim(rollerMotor, rollerConfig, DCMotor.getKrakenX60(1), 0.008, false);


        intakeSim = IntakeSimulation.OverTheBumperIntake(
                "Coral",
                SwerveConstants.SWERVE_DRIVE_SIMULATION,
                Meter.of(0.44),
                Meter.of(0.2625),
                IntakeSide.FRONT,
                1);
    }

    @Override
    public boolean hasCoral() {
        return intakeSim.getGamePiecesAmount() > 0;
    }

    @Override
    public void updatePeriodic() {
        super.updatePeriodic();

        motorSim.updateSim();
        

    }
}

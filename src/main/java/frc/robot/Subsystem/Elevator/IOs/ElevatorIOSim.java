package frc.robot.Subsystem.Elevator.IOs;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Subsystem.Elevator.ElevatorConstants;

public class ElevatorIOSim extends ElevatorIOReal {

    private final TalonFXSimState simState;
    private final ElevatorSim elevatorSim;

    public ElevatorIOSim() {
        super();

        // Initialize the sim state for the motor
        simState = new TalonFXSimState(masterMotor);

        // Gear ratio is motor rotations per elevator rotation (same as real)
        double gearing = ElevatorConstants.GEAR;

        // ElevatorSim expects: motor, gear ratio, carriage mass (kg), radius (m), min height (m), max height (m), simulate gravity, noise
        elevatorSim = new ElevatorSim(
            DCMotor.getKrakenX60(1), // 1 motor
            gearing,
            4.0, // Approximate mass of elevator carriage
            ElevatorConstants.SPROKET_PITCH_DIAMETER / 2.0,
            ElevatorConstants.MIN_HIGHT,
            ElevatorConstants.MAX_HIGHT,
            false,
            0
        );
    }

    @Override
    public void updatePeriodic() {
        // Apply 12V supply voltage to sim state
        simState.setSupplyVoltage(12.0);

        // Simulate the input voltage from control system
        double inputVolts = simState.getMotorVoltage();
        elevatorSim.setInputVoltage(inputVolts);

        // Step the simulation forward by 20 ms (typical loop time)
        elevatorSim.update(0.02);

        // Convert position from meters to motor rotor rotations
        // Distance traveled / (sprocket circumference) = elevator rotations
        // Elevator rotations * gear ratio = motor rotations
        double elevatorPositionMeters = elevatorSim.getPositionMeters();
        double elevatorRotations = elevatorPositionMeters / ElevatorConstants.SPROKET_CIRCUMFERENCE;
        double motorRotations = elevatorRotations * ElevatorConstants.GEAR;

        simState.setRawRotorPosition(motorRotations);

        // Velocity in m/s → elevator RPS → motor RPS
        double elevatorVelocityRPS = elevatorSim.getVelocityMetersPerSecond() / ElevatorConstants.SPROKET_CIRCUMFERENCE;
        double motorVelocityRPS = elevatorVelocityRPS * ElevatorConstants.GEAR;

        simState.setRotorVelocity(motorVelocityRPS);

        // Now call super to log and read values
        super.updatePeriodic();
    }
}

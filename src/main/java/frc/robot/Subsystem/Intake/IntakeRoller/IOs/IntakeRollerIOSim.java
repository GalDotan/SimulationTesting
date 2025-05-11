package frc.robot.Subsystem.Intake.IntakeRoller.IOs;

import com.ma5951.utils.ControlledMotors.Sim.TalonFXMotorSim;

import edu.wpi.first.math.system.plant.DCMotor;

public class IntakeRollerIOSim extends IntakeRollerIOReal {

    private TalonFXMotorSim motorSim;
    private static boolean hasCoral;

    public IntakeRollerIOSim() {
        super();

        motorSim = new TalonFXMotorSim(rollerMotor, rollerConfig, DCMotor.getKrakenX60(1), 0.001, false);
        hasCoral = false;

    }

    @Override
    public boolean hasCoral() {
        return hasCoral;
    }

    public static boolean setHasCoral(boolean has) {
        return hasCoral = has;
    }

    @Override
    public void updatePeriodic() {
        
        motorSim.updateSim();
        super.updatePeriodic();

    }
}

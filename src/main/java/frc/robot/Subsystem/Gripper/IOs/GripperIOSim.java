package frc.robot.Subsystem.Gripper.IOs;

import org.ironmaple.simulation.IntakeSimulation;
import com.ma5951.utils.ControlledMotors.Sim.TalonFXMotorSim;

import edu.wpi.first.math.system.plant.DCMotor;

public class GripperIOSim extends GripperIOReal {

    private TalonFXMotorSim motorSim;
    public static IntakeSimulation intakeSim;

    private static boolean hasCoral = false;


    public GripperIOSim() {
        super();

        motorSim = new TalonFXMotorSim(rollerMotor, rollerConfig, DCMotor.getKrakenX60(1), 0.008, false);


        
    }

    @Override
    public boolean hasCoral() {
        return hasCoral;
    }

    public static void setHasCoral(boolean has) {
        hasCoral = has;
    }

    @Override
    public void updatePeriodic() {
        super.updatePeriodic();

        motorSim.updateSim();
        

    }
}

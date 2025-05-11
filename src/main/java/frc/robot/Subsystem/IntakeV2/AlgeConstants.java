
package frc.robot.Subsystem.IntakeV2;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ma5951.utils.RobotControl.Subsystems.DeafultSystems.ConstantsClasses.RollerSystemConstants;
import com.ma5951.utils.RobotControl.Utils.Motor;
import com.ma5951.utils.RobotControl.Utils.Motor.Motors;
import com.ma5951.utils.RobotControl.Utils.Sensors.BaseSensor;

public class AlgeConstants {

    public static final RollerSystemConstants ALGE_INTAKE = new RollerSystemConstants(
            new Motor[] {
                    new Motor(Motors.KrakenX60, new TalonFX(33), InvertedValue.Clockwise_Positive,
                            "ALGE_INTAKE_ROLLER") },
            3,
            40,
            true,
            60,
            "Subsystems/IntakeAlge/",
            true,
            0.004,
            new BaseSensor[] {});



}
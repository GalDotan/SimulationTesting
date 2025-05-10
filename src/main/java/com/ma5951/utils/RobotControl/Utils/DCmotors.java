
package com.ma5951.utils.RobotControl.Utils;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DCmotors {

    public DCMotor motor;
    public int numOfMotors;

    private DCmotors(DCMotor motor, int numOfMotors) {
        this.motor = motor;
        this.numOfMotors = numOfMotors;
    }

    public static DCmotors getKrakenX60(int numOfMotors) {
        return new DCmotors(DCMotor.getKrakenX60(numOfMotors), numOfMotors);
    }

    public static DCmotors getFalcon500(int numOfMotors) {
        return new DCmotors(DCMotor.getFalcon500(numOfMotors), numOfMotors);
    }

    public static DCmotors getNEO(int numOfMotors) {
        return new DCmotors(DCMotor.getNEO(numOfMotors), numOfMotors);
    }

    public static DCmotors getNEO550(int numOfMotors) {
        return new DCmotors(DCMotor.getNeo550(numOfMotors), numOfMotors);
    }

    public static DCmotors getKrakenX60FOC(int numOfMotors) {
        return new DCmotors(DCMotor.getKrakenX60Foc(1), numOfMotors);
    }

    public static DCmotors getFalcon500FOC(int numOfMotors) {
        return new DCmotors(DCMotor.getFalcon500Foc(1), numOfMotors);
    }

    public enum Motor {
        KrakenX60(
            12, 7.09, 366, 2, Units.rotationsPerMinuteToRadiansPerSecond(6000)),
        Falcon500(
            12, 4.69, 257, 1.5, Units.rotationsPerMinuteToRadiansPerSecond(6380.0)),
        NEO(
            12, 2.6, 105, 1.8, Units.rotationsPerMinuteToRadiansPerSecond(5676)),
        NEO550(
        12, 0.97, 100, 1.4, Units.rotationsPerMinuteToRadiansPerSecond(11000.0)),
        KrakenX60FOC(
            12, 9.37, 483, 2, Units.rotationsPerMinuteToRadiansPerSecond(5800)),
        Falcon500FOC(
            12, 5.84, 304, 1.5, Units.rotationsPerMinuteToRadiansPerSecond(6080.0));

        public final double nominalVoltageVolts;
        public final double stallTorqueNewtonMeters;
        public final double stallCurrentAmps;
        public final double freeCurrentAmps;
        public final double freeSpeedRadPerSec;
        public final double rOhms;
        public final double KvRadPerSecPerVolt;
        public final double KtNMPerAmp;

        private Motor(
                double nominalVoltageVolts,
                double stallTorqueNewtonMeters,
                double stallCurrentAmps,
                double freeCurrentAmps,
                double freeSpeedRadPerSec) {
            this.nominalVoltageVolts = nominalVoltageVolts;
            this.stallTorqueNewtonMeters = stallTorqueNewtonMeters;
            this.stallCurrentAmps = stallCurrentAmps;
            this.freeCurrentAmps = freeCurrentAmps;
            this.freeSpeedRadPerSec = freeSpeedRadPerSec;

            this.rOhms = nominalVoltageVolts / this.stallCurrentAmps;
            this.KvRadPerSecPerVolt = freeSpeedRadPerSec / (nominalVoltageVolts - rOhms * this.freeCurrentAmps);
            this.KtNMPerAmp = this.stallTorqueNewtonMeters / this.stallCurrentAmps;
        }

    }

}

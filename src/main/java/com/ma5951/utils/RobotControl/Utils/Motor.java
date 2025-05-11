
package com.ma5951.utils.RobotControl.Utils;

import java.io.ObjectInputFilter.Status;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class Motor {

    public Motors motorType;
    public TalonFX talonFX;
    public InvertedValue direction;
    public String name;
    public StatusSignal<AngularVelocity> statusSignal;

    public Motor(Motors motorType, TalonFX talonFX, InvertedValue direction, String name) {
        this.motorType = motorType;
        this.talonFX = talonFX;
        this.direction = direction;
        this.name = name;
        statusSignal = talonFX.getVelocity();
    }

    public enum Motors {
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

        private Motors(
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

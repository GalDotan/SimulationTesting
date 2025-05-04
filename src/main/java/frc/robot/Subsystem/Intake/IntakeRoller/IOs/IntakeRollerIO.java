package frc.robot.Subsystem.Intake.IntakeRoller.IOs;

public interface IntakeRollerIO {


    boolean hasCoral();

    double getCurrent();

    double getAppliedVolts();

    double getVelocity();

    void setNeutralMode(boolean isBrake);

    void setVoltage(double volt);

    void updatePeriodic();
}

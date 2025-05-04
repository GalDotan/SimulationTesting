package frc.robot.Subsystem.Gripper.IOs;

public interface GripperIO {


    boolean hasCoral();

    double getCurrent();

    double getAppliedVolts();

    double getVelocity();

    void setNeutralMode(boolean isBrake);

    void setVoltage(double volt);

    void updatePeriodic();
}

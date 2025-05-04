package frc.robot.Subsystem.Intake.IntakeArm.IOs;

public interface IntakeArmIO {

    double getAbsolutePosition();

    double getCurrent();

    double getPosition();

    double getAppliedVolts();

    double getError();

    double getSetPoint();

    void resetPosition(double newPose);

    void updatePID(double Kp, double Ki, double Kd);

    void setNeutralMode(boolean isBrake);

    void setVoltage(double volt);

    void setAngle(double angle , double FEED_FORWARD);

    void updatePeriodic();
}

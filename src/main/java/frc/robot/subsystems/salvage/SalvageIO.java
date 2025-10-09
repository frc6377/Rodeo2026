package frc.robot.subsystems.salvage;

public interface SalvageIO {
    class SalvageIOInputs {
        public double armPositionDegrees = 0.0;
        public double armVelocityDegreesPerSec = 0.0;
        public double armCurrentAmps = 0.0;
        public double intakeCurrentAmps = 0.0;
        public boolean atSetpoint = false;
    }

    default void updateInputs(SalvageIOInputs inputs) {}

    default void setArmPosition(double degrees) {}

    default void setArmVoltage(double volts) {}

    default void stopArm() {}

    default void setIntakeSpeed(double percentOutput) {}

    default void stopIntake() {}
}

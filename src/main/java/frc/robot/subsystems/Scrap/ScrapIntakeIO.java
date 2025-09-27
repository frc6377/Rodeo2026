package frc.robot.subsystems.scrap;

public interface ScrapIntakeIO {
    public static class ScrapIntakeIOInputs {
        public double armPositionDegrees = 0.0;
        public double armVelocityDegreesPerSec = 0.0;
        public double armCurrentAmps = 0.0;
        public double rollerCurrentAmps = 0.0;
        public boolean atSetpoint = false;
    }

    default void updateInputs(ScrapIntakeIOInputs inputs) {}

    default void setArmPosition(double degrees) {}

    default void setRollerSpeed(double rpm) {}

    default void setArmVoltage(double volts) {}

    default void stopArm() {}

    default void setRollerVoltage(double volts) {}

    default void stopRoller() {}

    default void pivotUp() {}

    default void pivotDown() {}
}

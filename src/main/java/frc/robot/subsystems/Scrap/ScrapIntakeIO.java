package frc.robot.subsystems.scrap;

public interface ScrapIntakeIO {
    public static class ScrapIntakeIOInputs {
        public double armPositionDegrees = 0.0;
        public double armVelocityDegreesPerSec = 0.0;
        public double armCurrentAmps = 0.0;
        public double rollerCurrentAmps = 0.0;
        public boolean atSetpoint = false;
    }

    public void updateInputs(ScrapIntakeIOInputs inputs);

    public void setArmPosition(double degrees);

    public void setRollerSpeed(double rpm);

    public void setArmVoltage(double volts);

    public void stopArm();

    public void setRollerVoltage(double volts);

    public void stopRoller();
}

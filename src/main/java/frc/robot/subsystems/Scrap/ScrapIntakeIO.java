package frc.robot.subsystems.scrap;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;

public interface ScrapIntakeIO {
    public static class ScrapIntakeIOInputs {
        public double armPositionDegrees = 0.0;
        public double armVelocityDegreesPerSec = 0.0;
        public double armCurrentAmps = 0.0;
        public double rollerCurrentAmps = 0.0;
        public boolean atSetpoint = false;
    }

    default void updateInputs(ScrapIntakeIOInputs inputs) {}

    default Command setArmPosition(Angle degrees) {
        return null;
    }

    default void setArmSetpoint(Angle degrees) {}

    default void setRollerSpeed(double rpm) {}

    default void setArmPercent(double volts) {}

    default void gotoAngle() {}

    default void stopArm() {}

    default void setRollerVoltage(double volts) {}

    default void stopRoller() {}

    default void pivotUp() {}

    default void pivotDown() {}
}

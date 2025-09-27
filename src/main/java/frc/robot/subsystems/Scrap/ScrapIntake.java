package frc.robot.subsystems.scrap;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ScrapIntake extends SubsystemBase {
    private final ScrapIntakeIO io;
    private final ScrapIntakeIO.ScrapIntakeIOInputs inputs = new ScrapIntakeIO.ScrapIntakeIOInputs();

    public ScrapIntake(ScrapIntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        SmartDashboard.putNumber("ScrapIntake/Arm Position", inputs.armPositionDegrees);
        SmartDashboard.putNumber("ScrapIntake/Arm Current", inputs.armCurrentAmps);
        SmartDashboard.putBoolean("ScrapIntake/At Setpoint", inputs.atSetpoint);
    }

    public void goToPosition(double degrees) {
        io.setArmPosition(degrees);
    }

    public void setArmVoltage(double volts) {
        io.setArmVoltage(volts);
    }
}

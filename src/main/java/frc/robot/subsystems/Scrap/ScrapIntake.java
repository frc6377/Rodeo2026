package frc.robot.subsystems.scrap;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ScrapArmConstants;

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

    public Command intake() {
        return runOnce(() -> io.setArmVoltage(ScrapArmConstants.kArmMinAngle))
                .andThen(() -> io.setRollerVoltage(-6.0)); // TODO: refactor to be alongwith
    }
}

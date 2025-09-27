package frc.robot.subsystems.scrap;

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
    }

    public void goToPosition(double degrees) {
        io.setArmPosition(degrees);
    }
}

package frc.robot.subsystems.Scrap;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class ScrapIntake {
    private TalonSRX intakeMotor;
    private TalonSRX pivotMotor;

    public ScrapIntake() {
        intakeMotor = new TalonSRX(10); // Example ID
        pivotMotor = new TalonSRX(11); // Example ID
    }

    public void goToPivotPosition(double position) {
        // Code to move pivot to a specific position
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.set(TalonSRXControlMode.Current, speed);
    }
}

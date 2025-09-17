package frc.robot.subsystems.Scrap;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants.MotorIDs;

public class ScrapIntake {
    private TalonSRX intakeMotor;
    private TalonSRX pivotMotor;

    public ScrapIntake() {
        intakeMotor = new TalonSRX(MotorIDs.intakeMotorID);
        pivotMotor = new TalonSRX(MotorIDs.pivotMotorID);
    }

    public void goToPivotPosition(double position) {
        // Code to move pivot to a specific position
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.set(TalonSRXControlMode.Current, speed);
    }
}

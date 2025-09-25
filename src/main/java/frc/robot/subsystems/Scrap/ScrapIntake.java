package frc.robot.subsystems.scrap;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;

public class ScrapIntake extends SubsystemBase {
    private TalonSRX intakeMotor;
    private TalonSRX pivotMotor;

    private CANcoder pivotEncoder;

    public ScrapIntake() {
        intakeMotor = new TalonSRX(MotorIDs.intakeMotorID);
        pivotMotor = new TalonSRX(MotorIDs.pivotMotorID);
        pivotEncoder = new CANcoder(MotorIDs.pivotEncoderID);
    }

    public void goToPivotPosition(double position) {
        // Code to move pivot to a specific position
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.set(TalonSRXControlMode.Current, speed);
    }
}

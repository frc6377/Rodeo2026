package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants.MotorIDs;

public class SalvageManipulator {
    private TalonSRX pivotMotor;

    public SalvageManipulator() {
        pivotMotor = new TalonSRX(MotorIDs.salvagePivotMotorID); // Example motor ID
    }
}

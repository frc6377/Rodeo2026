package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MotorIDs;

public class SalvageManipulator {
    private TalonSRX pivotMotor;

    public SalvageManipulator() {
        pivotMotor = new TalonSRX(MotorIDs.salvagePivotMotorID); // Example motor ID
    }

    public Command pivotCommand(double speed) {
        return null;
    }
}

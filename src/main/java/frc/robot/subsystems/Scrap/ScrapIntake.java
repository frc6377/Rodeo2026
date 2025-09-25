package frc.robot.subsystems.scrap;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;

public class ScrapIntake extends SubsystemBase {
    private TalonSRX intakeMotor;
    private TalonSRX pivotMotor;

    private CANcoder pivotEncoder;

    public ScrapIntake() {
        intakeMotor = new TalonSRX(MotorIDs.intakeMotorID);
        pivotMotor = new TalonSRX(MotorIDs.pivotMotorID);
        pivotMotor.config_kP(0, 0.1);
        pivotMotor.config_kI(0, 0.0);
        pivotMotor.config_kD(0, 0.0);

        pivotEncoder = new CANcoder(MotorIDs.pivotEncoderID);
    }

    public Angle getPivotAngle() {
        return pivotEncoder.getPosition().getValue();
    }

    public Command goToPivotPosition(double position) {
        return Commands.run(
                () -> {
                    pivotMotor.set(TalonSRXControlMode.Position, position);
                },
                this);
    }

    public boolean atSetpoint(Angle setpoint) {
        return pivotEncoder.getPosition().equals(setpoint);
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.set(TalonSRXControlMode.Current, speed);
    }
}

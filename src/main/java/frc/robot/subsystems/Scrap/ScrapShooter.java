package frc.robot.subsystems.scrap;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;

public class ScrapShooter extends SubsystemBase {

    private TalonSRX shooterMotor1;
    private TalonSRX shooterMotor2;

    public ScrapShooter() {
        shooterMotor1 = new TalonSRX(MotorIDs.shooterMotor1ID);
        shooterMotor2 = new TalonSRX(MotorIDs.shooterMotor2ID);
    }

    public Command setShooterSpeed(double speed) {
        return Commands.run(
                () -> {
                    shooterMotor1.set(TalonSRXControlMode.Current, speed);
                    shooterMotor2.set(TalonSRXControlMode.Current, speed);
                },
                this);
    }

    public Command stopShooter() {
        return Commands.run(
                () -> {
                    shooterMotor1.set(TalonSRXControlMode.Current, 0);
                    shooterMotor2.set(TalonSRXControlMode.Current, 0);
                },
                this);
    }
}

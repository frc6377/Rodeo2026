package frc.robot.subsystems.scrap;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;
import frc.robot.Constants.MotorIDs;

public class ScrapIntakeReal implements ScrapIntakeIO {

    private TalonSRX intakeMotor;
    private TalonSRX pivotMotor;
    private CANcoder pivotEncoder;
    private double armSetpoint = 0.0;

    public ScrapIntakeReal() {
        intakeMotor = new TalonSRX(MotorIDs.intakeMotorID);
        pivotMotor = new TalonSRX(MotorIDs.pivotMotorID);
        pivotMotor.config_kP(0, 0.1);
        pivotMotor.config_kI(0, 0.0);
        pivotMotor.config_kD(0, 0.0);
        pivotEncoder = new CANcoder(MotorIDs.pivotEncoderID);
    }

    @Override
    public void updateInputs(ScrapIntakeIOInputs inputs) {
        inputs.armPositionDegrees = pivotEncoder.getPosition().getValue().in(Degrees);
        inputs.armVelocityDegreesPerSec = pivotEncoder.getVelocity().getValue().in(DegreesPerSecond);
        inputs.armCurrentAmps = pivotMotor.getStatorCurrent();
        inputs.rollerCurrentAmps = intakeMotor.getStatorCurrent();
        inputs.atSetpoint = Math.abs(inputs.armPositionDegrees - armSetpoint)
                < 1.0; // TODO: 1 degree tolerance -> maybe change later depending on real robot testing
    }

    public void setArmPosition(double degrees) {
        armSetpoint = degrees;
        pivotMotor.set(TalonSRXControlMode.Position, degrees);
    }

    public void setArmVoltage(double volts) {
        pivotMotor.set(TalonSRXControlMode.PercentOutput, volts / 12.0);
    }

    public void stopArm() {
        pivotMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
    }

    public void setRollerSpeed(double rpm) {
        intakeMotor.set(TalonSRXControlMode.Velocity, rpm);
    }

    public void setRollerVoltage(double volts) {
        intakeMotor.set(TalonSRXControlMode.PercentOutput, volts / 12.0);
    }

    public void stopRoller() {
        intakeMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
    }
}

package frc.robot.subsystems.salvage;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.SensorIDs;

public class SalvageReal implements SalvageIO {
    private TalonSRX intakeMotor;
    private TalonSRX armMotor;
    private CANcoder salvagePivotEncoder;
    private double armSetpoint = 0.0;

    public SalvageReal() {
        intakeMotor = new TalonSRX(MotorIDs.salvageMotor);
        armMotor = new TalonSRX(MotorIDs.salvageArmMotor);
        salvagePivotEncoder = new CANcoder(SensorIDs.salvagePivotEncoder);

        // Configure PID for arm motor
        armMotor.config_kP(0, 0.02);
        armMotor.config_kI(0, 0.0);
        armMotor.config_kD(0, 0.0);
    }

    @Override
    public void updateInputs(SalvageIOInputs inputs) {
        inputs.armPositionDegrees =
                salvagePivotEncoder.getAbsolutePosition().getValue().in(Degrees) * 360;
        inputs.armVelocityDegreesPerSec =
                salvagePivotEncoder.getVelocity().getValue().in(DegreesPerSecond);
        inputs.armCurrentAmps = armMotor.getStatorCurrent();
        inputs.intakeCurrentAmps = intakeMotor.getStatorCurrent();
        inputs.atSetpoint = Math.abs(inputs.armPositionDegrees - armSetpoint) < 2.0;
    }

    @Override
    public void setArmPosition(double degrees) {
        armSetpoint = degrees;
        armMotor.set(ControlMode.Position, degrees);
    }

    @Override
    public void setArmVoltage(double volts) {
        armMotor.set(ControlMode.PercentOutput, volts / 12.0);
    }

    @Override
    public void stopArm() {
        armMotor.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public void setIntakeSpeed(double percentOutput) {
        intakeMotor.set(ControlMode.PercentOutput, percentOutput);
    }

    @Override
    public void stopIntake() {
        intakeMotor.set(ControlMode.PercentOutput, 0.0);
    }
}

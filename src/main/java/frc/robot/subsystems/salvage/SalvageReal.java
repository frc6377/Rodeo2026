package frc.robot.subsystems.salvage;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.SalvageArmConstants;
import frc.robot.Constants.SensorIDs;

public class SalvageReal implements SalvageIO {
    private TalonSRX intakeMotor;
    private TalonSRX armMotor;
    private DutyCycleEncoder salvagePivotEncoder;
    private double armSetpoint = 0.0;

    public SalvageReal() {
        intakeMotor = new TalonSRX(MotorIDs.salvageMotor);
        armMotor = new TalonSRX(MotorIDs.salvageArmMotor);
        salvagePivotEncoder = new DutyCycleEncoder(SensorIDs.salvagePivotEncoder);

        // Configure PID for arm motor (Talon onboard PID - currently unused)
        armMotor.config_kP(0, SalvageArmConstants.TalonPID.kP);
        armMotor.config_kI(0, SalvageArmConstants.TalonPID.kI);
        armMotor.config_kD(0, SalvageArmConstants.TalonPID.kD);
    }

    @Override
    public void updateInputs(SalvageIOInputs inputs) {
        inputs.armPositionDegrees = salvagePivotEncoder.get();
        // inputs.armVelocityDegreesPerSec = salvagePivotEncoder.getVelocity().getValue().in(DegreesPerSecond);
        inputs.armCurrentAmps = armMotor.getStatorCurrent();
        inputs.intakeCurrentAmps = intakeMotor.getStatorCurrent();
        inputs.atSetpoint = Math.abs(inputs.armPositionDegrees - armSetpoint) < SalvageArmConstants.PID.tolerance;

        System.out.println("SalvageReal - Encoder: " + inputs.armPositionDegrees + " | Setpoint: " + armSetpoint);
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

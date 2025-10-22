package frc.robot.subsystems.salvage;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.SalvageArmConstants;
import frc.robot.Constants.SensorIDs;
import org.littletonrobotics.junction.Logger;

public class SalvageReal implements SalvageIO {
    private TalonSRX intakeMotor;
    private TalonSRX armMotor;
    private CANcoder salvagePivotEncoder;
    private double armSetpoint = 0.0;

    private PIDController pid;
    private ArmFeedforward armFF;

    public SalvageReal() {
        intakeMotor = new TalonSRX(MotorIDs.salvageMotor);
        armMotor = new TalonSRX(MotorIDs.salvageArmMotor);
        armMotor.setNeutralMode(NeutralMode.Brake);
        salvagePivotEncoder = new CANcoder(SensorIDs.salvagePivotEncoder);

        pid = new PIDController(1, 0, 2);

        armFF = new ArmFeedforward(
                SalvageArmConstants.Feedforward.kS,
                SalvageArmConstants.Feedforward.kG,
                SalvageArmConstants.Feedforward.kV,
                0.0); // kA - acceleration feedforward

        // Configure PID for arm motor (Talon onboard PID - currently unused)
        armMotor.config_kP(0, SalvageArmConstants.TalonPID.kP);
        armMotor.config_kI(0, SalvageArmConstants.TalonPID.kI);
        armMotor.config_kD(0, SalvageArmConstants.TalonPID.kD);
    }

    @Override
    public void updateInputs(SalvageIOInputs inputs) {
        // CANcoder returns rotations, convert to degrees
        inputs.armPositionDegrees =
                salvagePivotEncoder.getAbsolutePosition().getValue().in(Degrees);
        inputs.armVelocityDegreesPerSec =
                salvagePivotEncoder.getVelocity().getValue().in(DegreesPerSecond);

        // TalonSRX uses getOutputCurrent() instead of getStatorCurrent()
        inputs.armCurrentAmps = armMotor.getOutputCurrent();
        inputs.intakeCurrentAmps = intakeMotor.getOutputCurrent();

        inputs.atSetpoint = Math.abs(inputs.armPositionDegrees - armSetpoint) < SalvageArmConstants.PID.tolerance;

        System.out.println("SalvageReal - Encoder: " + inputs.armPositionDegrees + " | Setpoint: " + armSetpoint);
    }

    public double calculateFF() {
        double ff = armFF.calculate(
                salvagePivotEncoder.getAbsolutePosition().getValue().in(Degrees), armSetpoint);
        Logger.recordOutput("Salvage/Feedforward", ff);
        return ff;
    }

    @Override
    public void setArmPosition(double degrees) {
        armSetpoint = degrees;
        double pidOutput =
                pid.calculate(salvagePivotEncoder.getAbsolutePosition().getValueAsDouble(), degrees);
        double ffOutput = calculateFF();
        double totalOutput = pidOutput + ffOutput;
        setArmSpeed(totalOutput);
        Logger.recordOutput("Salvage/PIDOutput", pidOutput);
        Logger.recordOutput("Salvage/TotalOutput", totalOutput);
    }

    @Override
    public void setArmVoltage(double volts) {
        armMotor.set(ControlMode.PercentOutput, volts / 12.0);
    }

    @Override
    public void setArmSpeed(double degreesPerSec) {

        armMotor.set(TalonSRXControlMode.PercentOutput, degreesPerSec);
    }

    @Override
    public void setIntakeSpeed(double speed) {
        intakeMotor.set(ControlMode.PercentOutput, speed);
    }

    @Override
    public void stopArm() {
        // Stop the arm motor but keep the current setpoint
        // This allows gravity compensation to continue working
        armMotor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void stopIntake() {
        intakeMotor.set(ControlMode.PercentOutput, 0);
    }
}

package frc.robot.subsystems.scrap;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.ScrapArmConstants;
import org.littletonrobotics.junction.Logger;

public class ScrapIntakeReal implements ScrapIntakeIO {

    protected TalonSRX intakeMotor;
    protected TalonSRX pivotMotor;
    protected DutyCycleEncoder pivotEncoder;
    protected double armSetpoint = 0.0;

    private PIDController pidController;
    private ArmFeedforward armFF;

    public ScrapIntakeReal() {
        intakeMotor = new TalonSRX(MotorIDs.intakeMotorID);
        pivotMotor = new TalonSRX(MotorIDs.pivotMotorID);
        pivotMotor.config_kP(0, 0.1);
        pivotMotor.config_kI(0, 0.0);
        pivotMotor.config_kD(0, 0.0);
        pivotEncoder = new DutyCycleEncoder(MotorIDs.pivotEncoderID);

        pidController = new PIDController(ScrapArmConstants.PID.kP, ScrapArmConstants.PID.kI, ScrapArmConstants.PID.kD);

        armFF = new ArmFeedforward(
                ScrapArmConstants.FEEDFORWARD.kS,
                ScrapArmConstants.FEEDFORWARD.kG,
                ScrapArmConstants.FEEDFORWARD.kV,
                ScrapArmConstants.FEEDFORWARD.kA);
    }

    @Override
    public void updateInputs(ScrapIntakeIOInputs inputs) {
        inputs.armPositionDegrees = pivotEncoder.get();
        // inputs.armVelocityDegreesPerSec = pivotEncoder.getVelocity().getValue().in(DegreesPerSecond);
        inputs.armCurrentAmps = pivotMotor.getStatorCurrent();
        inputs.rollerCurrentAmps = intakeMotor.getStatorCurrent();
        inputs.atSetpoint = Math.abs(inputs.armPositionDegrees - armSetpoint)
                < 1.0; // TODO: 1 degree tolerance -> maybe change later depending on real robot testing
    }

    public double calculateFF() {
        double ff = armFF.calculate(pivotEncoder.get(), armSetpoint);
        Logger.recordOutput("ScrapIntake/Feedforward", ff);
        return ff;
    }

    @Override
    public Command setArmPosition(Angle degrees) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    pidController.setSetpoint(degrees.in(Degrees));
                }),
                Commands.run(() -> {
                    double output = pidController.calculate(pivotEncoder.get());
                    pivotMotor.set(ControlMode.PercentOutput, output);
                    Logger.recordOutput("ScrapIntake/Output", output);
                })); // TODO: ADD ARM FEEDFORWARD
    }

    @Override
    public void setArmSetpoint(Angle degrees) {
        System.out.println("Setting arm setpoint to: " + degrees.in(Degrees) + " degrees");
        armSetpoint = degrees.in(Degrees);
        pidController.setSetpoint(armSetpoint);
        double output = pidController.calculate(pivotEncoder.get());
        pivotMotor.set(ControlMode.PercentOutput, output);
        Logger.recordOutput("ScrapIntake/Setpoint", armSetpoint);
        Logger.recordOutput("ScrapIntake/Output", output);
    }

    @Override
    public void setArmPercent(double percent) {
        pivotMotor.set(TalonSRXControlMode.PercentOutput, percent);
    }

    @Override
    public void stopArm() {
        pivotMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
    }

    @Override
    public void setRollerSpeed(double rpm) {
        intakeMotor.set(TalonSRXControlMode.Velocity, rpm);
    }

    @Override
    public void setRollerVoltage(double volts) {
        intakeMotor.set(TalonSRXControlMode.PercentOutput, volts / 12.0);
    }

    @Override
    public void stopRoller() {
        intakeMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
    }

    @Override
    public void pivotUp() {
        if (armSetpoint < ScrapArmConstants.kArmMaxAngle.in(Degrees)) {
            armSetpoint += 1.0;
        }
    }

    @Override
    public void pivotDown() {
        if (armSetpoint > ScrapArmConstants.kArmMinAngle.in(Degrees)) {
            armSetpoint -= 1.0;
        }
    }
}

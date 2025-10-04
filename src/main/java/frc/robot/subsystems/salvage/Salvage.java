package frc.robot.subsystems.salvage;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.SensorIDs;

public class Salvage extends SubsystemBase {
    private TalonSRX intakeMotor;
    private TalonSRX armMotor;
    private final CANcoder salvagePivotEncoder;
    private final PIDController armPIDController;

    public Salvage() {
        intakeMotor = new TalonSRX(MotorIDs.salvageMotor);
        armMotor = new TalonSRX(MotorIDs.salvageArmMotor);
        salvagePivotEncoder = new CANcoder(SensorIDs.salvagePivotEncoder);

        armPIDController = new PIDController(0.02, 0.0, 0.0);
        armPIDController.setTolerance(2.0);
        armPIDController.enableContinuousInput(0, 360);
    }

    public Angle getCurrentAngle() {
        return Degrees.of(salvagePivotEncoder.getAbsolutePosition().getValueAsDouble() * 360);
    }

    public Command intakeCommand() {
        return Commands.startEnd(
                () -> intakeMotor.set(ControlMode.PercentOutput, 1.0),
                () -> intakeMotor.set(ControlMode.PercentOutput, 0.0),
                this);
    }

    public Command outtakeCommand() {
        return Commands.startEnd(
                () -> intakeMotor.set(ControlMode.PercentOutput, -1.0),
                () -> intakeMotor.set(ControlMode.PercentOutput, 0.0),
                this);
    }

    // 3 setpoints: intake, stow, frieght

    // STOW = 63.244319
    // FREIGHT = 43.750137
    public enum Setpoint {
        INTAKE(Degrees.of(0)),
        STOW(Degrees.of(63.244319)),
        FREIGHT(Degrees.of(43.750137));

        private final Angle angle;

        Setpoint(Angle angle) {
            this.angle = angle;
        }

        public Angle getAngle() {
            return angle;
        }
    }

    public Command moveArmCommand(Setpoint setpoint) {
        return Commands.run(
                        () -> {
                            double targetAngle = setpoint.getAngle().in(Degrees);
                            double currentAngle = getCurrentAngle().in(Degrees);
                            double output = armPIDController.calculate(currentAngle, targetAngle);
                            armMotor.set(ControlMode.PercentOutput, output);
                        },
                        this)
                .until(() -> armPIDController.atSetpoint());
    }

    public Command holdArmPositionCommand() {
        return Commands.run(
                () -> {
                    double currentAngle = getCurrentAngle().in(Degrees);
                    double output = armPIDController.calculate(currentAngle);
                    armMotor.set(ControlMode.PercentOutput, output);
                },
                this);
    }

    public void stopArm() {
        armMotor.set(ControlMode.PercentOutput, 0.0);
    }

    public void stopIntake() {
        intakeMotor.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Salvage Arm Angle", getCurrentAngle().in(Degrees));
    }
}

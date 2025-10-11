package frc.robot.subsystems.salvage;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Salvage extends SubsystemBase {
    private final SalvageIO io;
    private final SalvageIO.SalvageIOInputs inputs = new SalvageIO.SalvageIOInputs();
    private final PIDController armPIDController;

    public Salvage(SalvageIO io) {
        this.io = io;
        armPIDController = new PIDController(1, 0.0, 0.0);
        armPIDController.setTolerance(2.0);
        armPIDController.enableContinuousInput(0, 360);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        SmartDashboard.putNumber("Salvage/Arm Position", inputs.armPositionDegrees);
        SmartDashboard.putNumber("Salvage/Arm Current", inputs.armCurrentAmps);
        SmartDashboard.putNumber("Salvage/Intake Current", inputs.intakeCurrentAmps);
        SmartDashboard.putBoolean("Salvage/At Setpoint", inputs.atSetpoint);
    }

    public Angle getCurrentAngle() {
        return Degrees.of(inputs.armPositionDegrees);
    }

    public Command intakeCommand() {
        return Commands.startEnd(() -> io.setIntakeSpeed(1.0), () -> io.stopIntake(), this);
    }

    public Command outtakeCommand() {
        return Commands.startEnd(() -> io.setIntakeSpeed(-1.0), () -> io.stopIntake(), this);
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
                            io.setArmVoltage(output * 12.0);
                        },
                        this)
                .until(() -> armPIDController.atSetpoint());
    }

    public Command holdArmPositionCommand() {
        return Commands.run(
                () -> {
                    double currentAngle = getCurrentAngle().in(Degrees);
                    double output = armPIDController.calculate(currentAngle);
                    io.setArmVoltage(output * 12.0);
                },
                this);
    }

    public void stopArm() {
        io.stopArm();
    }

    public void stopIntake() {
        io.stopIntake();
    }
}

package frc.robot.subsystems.salvage;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SalvageArmConstants;

public class Salvage extends SubsystemBase {
    private final SalvageIO io;
    private final SalvageIO.SalvageIOInputs inputs = new SalvageIO.SalvageIOInputs();
    private final PIDController armPIDController;

    public Salvage(SalvageIO io) {
        this.io = io;
        armPIDController =
                new PIDController(SalvageArmConstants.PID.kP, SalvageArmConstants.PID.kI, SalvageArmConstants.PID.kD);
        armPIDController.setTolerance(SalvageArmConstants.PID.tolerance);
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
        // This command is designed to be used with .whileTrue()
        // While button is held: move to intake, hold position, run roller
        // When button released: the finallyDo will stop everything, then need separate command to return to stow
        return Commands.runEnd(
                        () -> {
                            // Move to intake position and run roller
                            double targetAngle = Setpoint.INTAKE.getAngle().in(Degrees);
                            double currentAngle = getCurrentAngle().in(Degrees);
                            double output = armPIDController.calculate(currentAngle, targetAngle);
                            io.setArmVoltage(output * 12.0);
                            io.setIntakeSpeed(1.0);
                        },
                        () -> {
                            // When button released: stop roller and arm
                            io.stopIntake();
                            io.stopArm();
                        })
                .andThen(moveArmCommand(Setpoint.STOW)) // Return to stow after button released
                .withName("SalvageIntake");
    }

    public Command outtakeCommand() {
        return Commands.startEnd(() -> io.setIntakeSpeed(-1.0), () -> io.stopIntake(), this);
    }

    // 3 setpoints: intake, stow, freight
    public enum Setpoint {
        INTAKE(SalvageArmConstants.kArmIntakeAngle),
        STOW(SalvageArmConstants.kArmStowAngle),
        FREIGHT(SalvageArmConstants.kArmFreightAngle);

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

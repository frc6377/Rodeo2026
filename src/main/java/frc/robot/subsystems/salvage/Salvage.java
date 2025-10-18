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
    private int currentSetpointIndex = 0;

    public Salvage(SalvageIO io) {
        this.io = io;
        armPIDController =
                new PIDController(SalvageArmConstants.PID.kP, SalvageArmConstants.PID.kI, SalvageArmConstants.PID.kD);
        armPIDController.setTolerance(SalvageArmConstants.PID.tolerance);
        // No continuous input - arm doesn't do full rotations

        // Set default command to hold arm at current position with gravity compensation
        setDefaultCommand(holdArmPositionCommand());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        SmartDashboard.putNumber("Salvage/Arm Position", inputs.armPositionDegrees);
        SmartDashboard.putNumber("Salvage/Arm Current", inputs.armCurrentAmps);
        SmartDashboard.putNumber("Salvage/Intake Current", inputs.intakeCurrentAmps);
        SmartDashboard.putBoolean("Salvage/At Setpoint", inputs.atSetpoint);

        // Always apply gravity compensation to prevent arm from falling
        // Commands will add their own control output on top of this
        double currentAngle = getCurrentAngle().in(Degrees);
        double gravityCompensation = calculateFeedforward(currentAngle);
        SmartDashboard.putNumber("Salvage/Gravity Compensation", gravityCompensation);
    }

    public Angle getCurrentAngle() {
        return Degrees.of(inputs.armPositionDegrees);
    }

    /**
     * Calculate feedforward voltage to counteract gravity
     *
     * @param angleDegrees The current arm angle in degrees
     * @return The feedforward voltage needed
     */
    private double calculateFeedforward(double angleDegrees) {
        // Gravity compensation: kG * cos(angle)
        // This provides maximum support when horizontal (90°) and zero when vertical (0°)
        double angleRadians = Math.toRadians(angleDegrees);
        return SalvageArmConstants.Feedforward.kG * Math.cos(angleRadians);
    }

    public Command intakeCommand() {
        // This command is designed to be used with .whileTrue()
        // While button is held: move to intake, hold position, run roller
        // When button released: stop roller but hold arm position with gravity compensation
        return Commands.runEnd(
                        () -> {
                            // Move to intake position and run roller
                            double targetAngle = Setpoint.INTAKE.getAngle().in(Degrees);
                            double currentAngle = getCurrentAngle().in(Degrees);
                            double pidOutput = armPIDController.calculate(currentAngle, targetAngle);
                            double feedforward = calculateFeedforward(currentAngle);
                            io.setArmVoltage((pidOutput + feedforward) * 12.0);
                            io.setIntakeSpeed(1.0);
                        },
                        () -> {
                            // When button released: stop roller, hold arm position
                            io.stopIntake();
                        },
                        this)
                .withName("SalvageIntake");
    }

    public Command outtakeCommand() {
        return Commands.startEnd(() -> io.setIntakeSpeed(-1.0), () -> io.stopIntake(), this);
    }

    public Command salvageScore() {
        // This command is designed to be used with .whileTrue()
        // While button is held: move to freight position (if at intake), hold position, run roller outward
        // When button released: stop roller but hold arm position with gravity compensation
        return Commands.runEnd(
                        () -> {
                            double currentAngle = getCurrentAngle().in(Degrees);
                            double intakeAngle = Setpoint.INTAKE.getAngle().in(Degrees);
                            double freightAngle = Setpoint.FREIGHT.getAngle().in(Degrees);

                            // Determine target angle: if at intake, move to freight; otherwise stay at current angle
                            double targetAngle;
                            if (Math.abs(currentAngle - intakeAngle) < SalvageArmConstants.PID.tolerance) {
                                targetAngle = freightAngle;
                            } else {
                                targetAngle = currentAngle; // Hold current position
                            }

                            double pidOutput = armPIDController.calculate(currentAngle, targetAngle);
                            double feedforward = calculateFeedforward(currentAngle);
                            io.setArmVoltage((pidOutput + feedforward) * 12.0);
                            io.setIntakeSpeed(-1.0); // Negative for outtake
                        },
                        () -> {
                            // When button released: stop roller, hold arm position
                            io.stopIntake();
                        },
                        this)
                .withName("SalvageScore");
    }

    // 2 setpoints: intake, freight
    public enum Setpoint {
        INTAKE(SalvageArmConstants.kArmIntakeAngle),
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
        return Commands.runOnce(() -> {
                    // Reset the PID controller for a fresh start
                    armPIDController.reset();
                })
                .andThen(Commands.run(
                                () -> {
                                    double targetAngle = setpoint.getAngle().in(Degrees);
                                    double currentAngle = getCurrentAngle().in(Degrees);
                                    double pidOutput = armPIDController.calculate(currentAngle, targetAngle);
                                    double feedforward = calculateFeedforward(currentAngle);
                                    io.setArmVoltage((pidOutput + feedforward) * 12.0);
                                },
                                this)
                        .until(() -> armPIDController.atSetpoint()));
    }

    public Command holdArmPositionCommand() {
        return Commands.run(
                () -> {
                    double currentAngle = getCurrentAngle().in(Degrees);
                    // Hold current position with gravity compensation
                    double pidOutput = armPIDController.calculate(currentAngle);
                    double feedforward = calculateFeedforward(currentAngle);
                    io.setArmVoltage((pidOutput + feedforward) * 12.0);
                },
                this);
    }

    public Command cycleSetpointsCommand() {
        return Commands.sequence(
                        Commands.runOnce(() -> System.out.println("Moving to INTAKE (0°)")),
                        moveArmCommand(Setpoint.INTAKE).withTimeout(3),
                        Commands.waitSeconds(1),
                        Commands.runOnce(() -> System.out.println("Moving to FREIGHT (43.75°)")),
                        moveArmCommand(Setpoint.FREIGHT).withTimeout(3),
                        Commands.waitSeconds(1))
                .repeatedly()
                .withName("Cycle Salvage Setpoints");
    }

    public Command nextSetpointCommand() {
        return Commands.runOnce(() -> {
                    Setpoint[] setpoints = Setpoint.values();
                    currentSetpointIndex = (currentSetpointIndex + 1) % setpoints.length;
                    Setpoint nextSetpoint = setpoints[currentSetpointIndex];
                    System.out.println("Moving to " + nextSetpoint.name() + " ("
                            + nextSetpoint.getAngle().in(Degrees) + "°)");
                })
                .andThen(moveArmCommand(Setpoint.values()[currentSetpointIndex]))
                .withName("Next Setpoint");
    }

    public void stopArm() {
        io.stopArm();
    }

    public void stopIntake() {
        io.stopIntake();
    }
}

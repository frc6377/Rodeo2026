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
    private Setpoint targetSetpoint = Setpoint.INTAKE; // Start at INTAKE

    public Salvage(SalvageIO io) {
        this.io = io;
        armPIDController =
                new PIDController(SalvageArmConstants.PID.kP, SalvageArmConstants.PID.kI, SalvageArmConstants.PID.kD);
        armPIDController.setTolerance(SalvageArmConstants.PID.tolerance);
        // No continuous input - arm doesn't do full rotations

        // Set default command to hold at the target setpoint (INTAKE or FREIGHT)
        setDefaultCommand(holdSetpointCommand());
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
        // Just run the intake roller - arm position is independent
        return Commands.startEnd(() -> io.setIntakeSpeed(1.0), () -> io.stopIntake(), this)
                .withName("SalvageIntakeRoller");
    }

    public Command outtakeCommand() {
        // Just run the outtake roller - arm position is independent
        return Commands.startEnd(() -> io.setIntakeSpeed(-1.0), () -> io.stopIntake(), this)
                .withName("SalvageOuttakeRoller");
    }

    public Command salvageScore() {
        // This command is designed to be used with .whileTrue()
        // While button is held: move to freight position (if at intake), hold position, run roller outward
        // When button released: stop roller but hold arm position with gravity compensation
        return Commands.runEnd(
                        () -> {
                            moveArmCommand(Setpoint.FREIGHT).execute();
                            io.setIntakeSpeed(-1.0); // Negative for outtake
                        },
                        () -> {
                            // When button released: stop roller, hold arm position
                            io.stopIntake();
                        },
                        this)
                .withName("SalvageScore");
    }

    public Command salvageIntake() {
        return Commands.runEnd(
                        () -> {
                            moveArmCommand(Setpoint.INTAKE).execute();
                            io.setIntakeSpeed(1.0); // Positive for intake
                        },
                        () -> {
                            // When button released: stop roller, hold arm position
                            io.stopIntake();
                        },
                        this)
                .withName("SalvageIntake");
    }

    // 2 setpoints: intake, freight
    public enum Setpoint {
        INTAKE(SalvageArmConstants.kArmIntakeAngle),
        FREIGHT(SalvageArmConstants.kArmFreightAngle),
        ZERO(Degrees.of(0));

        private final Angle angle;

        Setpoint(Angle angle) {
            this.angle = angle;
        }

        public Angle getAngle() {
            return angle;
        }
    }

    public Command moveArmCommand(Setpoint setpoint, boolean force) {
        // Determine which setpoint we're closer to
        targetSetpoint = setpoint; // Update target setpoint for default command
        double currentAngle = getCurrentAngle().in(Degrees);
        double intakeAngle = Setpoint.INTAKE.getAngle().in(Degrees);
        double freightAngle = Setpoint.FREIGHT.getAngle().in(Degrees);

        double distanceToIntake = Math.abs(currentAngle - intakeAngle);
        double distanceToFreight = Math.abs(currentAngle - freightAngle);

        Setpoint closerSetpoint = distanceToIntake < distanceToFreight ? Setpoint.INTAKE : Setpoint.FREIGHT;

        if (!force && closerSetpoint == setpoint && distanceToIntake < SalvageArmConstants.PID.tolerance) {
            return Commands.none();
        }

        return toggleArmPositionCommand();
    } 

    /**
     * Toggle between INTAKE (0°) and FREIGHT (43.75°) Simple: just toggle the state variable and the default command
     * will handle movement
     */
    public Command toggleArmPositionCommand() {
        return Commands.runOnce(() -> {
                    // Toggle the target setpoint
                    if (targetSetpoint == Setpoint.INTAKE) {
                        targetSetpoint = Setpoint.FREIGHT;
                    } else {
                        targetSetpoint = Setpoint.INTAKE;
                    }

                    double currentAngle = getCurrentAngle().in(Degrees);
                    double intakeAngle = Setpoint.INTAKE.getAngle().in(Degrees);
                    double freightAngle = Setpoint.FREIGHT.getAngle().in(Degrees);

                    double distanceToIntake = Math.abs(currentAngle - intakeAngle);
                    double distanceToFreight = Math.abs(currentAngle - freightAngle);

                    targetSetpoint = distanceToIntake < distanceToFreight ? Setpoint.INTAKE : Setpoint.FREIGHT;

                    
                })
                .withName("Toggle Salvage Arm");
    }

    /** Default command - always drives arm to the target setpoint (INTAKE or FREIGHT) */
    public Command holdSetpointCommand() {
        return Commands.run(
                () -> {
                    double targetAngle = targetSetpoint.getAngle().in(Degrees);
                    double currentAngle = getCurrentAngle().in(Degrees);
                    double pidOutput = armPIDController.calculate(currentAngle, targetAngle);
                    double feedforward = calculateFeedforward(currentAngle);
                    double totalVoltage = (pidOutput + feedforward) * 12.0;

                    SmartDashboard.putNumber("Salvage/Target Setpoint", targetAngle);
                    SmartDashboard.putNumber("Salvage/PID Output", pidOutput);
                    SmartDashboard.putNumber("Salvage/Total Voltage", totalVoltage);

                    io.setArmVoltage(totalVoltage);
                },
                this);
    }

    public Command pivotUpCommand() {
        return Commands.startEnd(() -> io.setArmSpeed(0.5), () -> io.stopArm(), this)
                .withName("SalvagePivotUp");
    }

    public Command pivotDownCommand() {
        return Commands.startEnd(() -> io.setArmSpeed(-0.5), () -> io.stopArm(), this)
                .withName("SalvagePivotDown");
    }

    public void stopArm() {
        io.stopArm();
    }

    public void stopIntake() {
        io.stopIntake();
    }
}

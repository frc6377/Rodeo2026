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
        // Just run the intake roller - arm position is independent
        return Commands.startEnd(() -> io.setIntakeSpeed(1.0), () -> io.stopIntake(), this)
                .withName("SalvageIntakeRoller");
    }

    public Command outtakeCommand() {
        // Just run the outtake roller - arm position is independent
        return Commands.startEnd(() -> io.setIntakeSpeed(-1.0), () -> io.stopIntake(), this)
                .withName("SalvageOuttakeRoller");
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
                    double targetAngle = setpoint.getAngle().in(Degrees);
                    System.out.println("============================================");
                    System.out.println("MOVING ARM TO: " + setpoint.name());
                    System.out.println("TARGET ANGLE: " + targetAngle + "°");
                    System.out.println("CURRENT ANGLE: " + getCurrentAngle().in(Degrees) + "°");
                    System.out.println("============================================");
                    armPIDController.reset();
                })
                .andThen(Commands.run(
                                () -> {
                                    double targetAngle = setpoint.getAngle().in(Degrees);
                                    double currentAngle = getCurrentAngle().in(Degrees);
                                    double pidOutput = armPIDController.calculate(currentAngle, targetAngle);
                                    double feedforward = calculateFeedforward(currentAngle);
                                    double totalVoltage = (pidOutput + feedforward) * 12.0;
                                    SmartDashboard.putNumber("Salvage/Target Angle", targetAngle);
                                    SmartDashboard.putNumber("Salvage/PID Output", pidOutput);
                                    SmartDashboard.putNumber("Salvage/Total Voltage", totalVoltage);
                                    io.setArmVoltage(totalVoltage);
                                },
                                this)
                        .until(() -> armPIDController.atSetpoint()));
    }

    /** Toggle between INTAKE and FREIGHT positions - determines target based on current position */
    public Command toggleArmPositionCommand() {
        return Commands.either(
                moveArmCommand(Setpoint.FREIGHT),
                moveArmCommand(Setpoint.INTAKE),
                () -> {
                    // If closer to INTAKE, go to FREIGHT. Otherwise go to INTAKE
                    double currentAngle = getCurrentAngle().in(Degrees);
                    boolean goToFreight = Math.abs(currentAngle - Setpoint.INTAKE.getAngle().in(Degrees)) < 
                           Math.abs(currentAngle - Setpoint.FREIGHT.getAngle().in(Degrees));
                    System.out.println("Current angle: " + currentAngle + "° -> Going to " + 
                        (goToFreight ? "FREIGHT" : "INTAKE"));
                    return goToFreight;
                }
        ).withName("Toggle Salvage Arm");
    }

    public Command holdArmPositionCommand() {
        return Commands.run(
                () -> {
                    double currentAngle = getCurrentAngle().in(Degrees);
                    // Hold current position with gravity compensation
                    // Use current angle as setpoint so PID tries to hold this position
                    double pidOutput = armPIDController.calculate(currentAngle, currentAngle);
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

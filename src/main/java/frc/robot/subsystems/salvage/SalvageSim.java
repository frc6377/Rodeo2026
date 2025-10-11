package frc.robot.subsystems.salvage;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class SalvageSim extends SalvageReal {
    private SingleJointedArmSim armSim;
    private LoggedMechanism2d armMechanism2d;
    private LoggedMechanismRoot2d root;
    private LoggedMechanismLigament2d baseMech;
    private final ShuffleboardTab salvageTab = Shuffleboard.getTab("Salvage");

    private double armSetpoint = 0.0;

    // Salvage arm simulation constants - adjust these based on your actual arm
    private static final double kArmGearing = 100.0;
    private static final double kArmMOI = 0.5; // kg*m^2
    private static final double kArmLength = 0.5; // meters
    private static final double kArmMinAngle = 0.0; // degrees
    private static final double kArmMaxAngle = 90.0; // degrees

    public SalvageSim() {
        armSim = new SingleJointedArmSim(
                DCMotor.getCIM(1),
                kArmGearing,
                kArmMOI,
                kArmLength,
                Units.degreesToRadians(kArmMinAngle),
                Units.degreesToRadians(kArmMaxAngle),
                true,
                Units.degreesToRadians(0));

        armMechanism2d = new LoggedMechanism2d(1, 1);
        root = armMechanism2d.getRoot("Salvage Arm Sim", 0, 0);
        baseMech = root.append(
                new LoggedMechanismLigament2d("Salvage Arm Sim", kArmLength, 0, 20, new Color8Bit(Color.kGreen)));
        salvageTab.add("Salvage Arm Mech", armMechanism2d);
    }

    @Override
    public void updateInputs(SalvageIOInputs inputs) {
        // PID constants - tune these
        final double kP = 0.02;
        final double kD = 0.01;

        // Calculate position error
        double currentAngle = Units.radiansToDegrees(armSim.getAngleRads());
        double error = armSetpoint - currentAngle;
        double velocity = Units.radiansToDegrees(armSim.getVelocityRadPerSec());

        // Calculate output voltage using PD control
        double voltage = error * kP - velocity * kD;
        voltage = MathUtil.clamp(voltage, -12.0, 12.0);

        // Apply voltage and update sim
        armSim.setInput(voltage);
        armSim.update(0.02);

        // Update mechanism display
        baseMech.setAngle(currentAngle);

        // Update telemetry
        inputs.armPositionDegrees = currentAngle;
        inputs.armVelocityDegreesPerSec = velocity;
        inputs.armCurrentAmps = armSim.getCurrentDrawAmps();
        inputs.intakeCurrentAmps = 2.0; // Simulated intake current
        inputs.atSetpoint = Math.abs(error) < 2.0;
    }

    @Override
    public void setArmPosition(double degrees) {
        armSetpoint = degrees;
    }

    @Override
    public void setArmVoltage(double volts) {
        armSim.setInput(volts);
    }

    @Override
    public void stopArm() {
        armSim.setInput(0.0);
    }

    @Override
    public void setIntakeSpeed(double percentOutput) {
        // Simulation - no physical intake motor to control
    }

    @Override
    public void stopIntake() {
        // Simulation - no physical intake motor to control
    }
}

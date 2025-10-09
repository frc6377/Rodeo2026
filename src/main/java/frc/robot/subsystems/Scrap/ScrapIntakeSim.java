package frc.robot.subsystems.scrap;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.ScrapArmConstants;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ScrapIntakeSim implements ScrapIntakeIO {

    private SingleJointedArmSim armSim;
    private LoggedMechanism2d armMechanism2d;
    private LoggedMechanismRoot2d root;
    private LoggedMechanismLigament2d baseMech;
    private final ShuffleboardTab armTab = Shuffleboard.getTab("Arm");

    private double armSetpoint = 0.0;

    public ScrapIntakeSim() {
        armSim = new SingleJointedArmSim(
                ScrapArmConstants.kArmMotor,
                ScrapArmConstants.kArmGearing,
                ScrapArmConstants.kArmMOI,
                ScrapArmConstants.kArmScoringAngle,
                ScrapArmConstants.kArmMinAngle,
                ScrapArmConstants.kArmMaxAngle,
                true,
                Units.degreesToRadians(-90));

        armMechanism2d = new LoggedMechanism2d(1, 1);
        root = armMechanism2d.getRoot("Arm Sim", 0, 0);
        baseMech = root.append(new LoggedMechanismLigament2d(
                "Arm Sim",
                ScrapArmConstants.kArmLength,
                ScrapArmConstants.kArmBaseAngle,
                20,
                new Color8Bit(Color.kBlue)));
        armTab.add("Arm Mech", armMechanism2d);
    }

    public void updateInputs(ScrapIntakeIOInputs inputs) {
        // PID constants - tune these
        final double kP = 0.1;
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
        baseMech.setAngle(armSetpoint);
        armSim.update(0.02);

        // Update mechanism display
        baseMech.setAngle(currentAngle);

        // Update telemetry
        inputs.armPositionDegrees = currentAngle;
        inputs.armVelocityDegreesPerSec = velocity;
        inputs.armCurrentAmps = armSim.getCurrentDrawAmps();
        inputs.atSetpoint = Math.abs(error) < 2.0;
    }

    @Override
    public void setArmPosition(double degrees) {
        System.out.println("Sim setArmPosition called with: " + degrees);
        armSetpoint = degrees;
        baseMech.setAngle(armSetpoint);
    }

    @Override
    public void setArmVoltage(double volts) {
        System.out.println("Sim setArmVoltage called with: " + volts);
        armSim.setInput(volts);
    }

    @Override
    public void stopArm() {
        armSim.setInput(0.0);
    }

    @Override
    public void setRollerSpeed(double rpm) {
        // None for simulation
    }

    @Override
    public void setRollerVoltage(double volts) {
        // None for simulation
    }

    @Override
    public void stopRoller() {
        // None for simulation
    }

    @Override
    public void pivotUp() {
        if (armSetpoint < ScrapArmConstants.kArmMaxAngle) {
            armSetpoint += 1.0;
        }
    }

    @Override
    public void pivotDown() {
        if (armSetpoint > ScrapArmConstants.kArmMinAngle) {
            armSetpoint -= 1.0;
        }
    }
}

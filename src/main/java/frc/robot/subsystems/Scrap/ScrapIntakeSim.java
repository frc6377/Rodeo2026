package frc.robot.subsystems.scrap;

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
                "Arm Sim", ScrapArmConstants.kArmLength, ScrapArmConstants.kArmBaseAngle, 20, new Color8Bit(Color.kBlue)));
        armTab.add("Arm Mech", armMechanism2d);
    }

    public void updateInputs(ScrapIntakeIOInputs inputs) {
        armSim.update(0.02);

        inputs.armPositionDegrees = Units.radiansToDegrees(armSim.getAngleRads());
        inputs.armVelocityDegreesPerSec = Units.radiansToDegrees(armSim.getVelocityRadPerSec());
        inputs.armCurrentAmps = armSim.getCurrentDrawAmps();
        inputs.atSetpoint = Math.abs(inputs.armPositionDegrees - armSetpoint) < 2.0;
    }

    @Override
    public void setArmPosition(double degrees) {
        armSetpoint = degrees;
        baseMech.setAngle(degrees);
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

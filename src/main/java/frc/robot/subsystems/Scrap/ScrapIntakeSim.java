package frc.robot.subsystems.scrap;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.ArmConstants;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ScrapIntakeSim implements ScrapIntakeIO {

    private SingleJointedArmSim armSim;
    private LoggedMechanism2d armMechanism2d;
    private LoggedMechanismRoot2d root;
    private LoggedMechanismLigament2d baseMech;
    private LoggedMechanismLigament2d scoringMech;
    private final ShuffleboardTab armTab = Shuffleboard.getTab("Arm");

    public ScrapIntakeSim() {
        armSim = new SingleJointedArmSim(
                ArmConstants.kArmMotor,
                ArmConstants.kArmGearing,
                ArmConstants.kArmMOI,
                ArmConstants.kArmScoringAngle,
                ArmConstants.kArmMinAngle,
                ArmConstants.kArmMaxAngle,
                true,
                Units.degreesToRadians(-90));
        armMechanism2d = new LoggedMechanism2d(1, 1);
        root = armMechanism2d.getRoot("Arm Sim", 0, 0);
        baseMech = root.append(new LoggedMechanismLigament2d(
                "Arm Sim", ArmConstants.kArmLength, ArmConstants.kArmBaseAngle, 20, new Color8Bit(Color.kBlue)));
        armTab.add("Arm Mech", armMechanism2d);
    }

    public void setArmPosition(double degrees) {}

    public void setArmVoltage(double volts) {}

    public void stopArm() {}

    public void setRollerSpeed(double rpm) {}

    public void setRollerVoltage(double volts) {}

    public void stopRoller() {}
}

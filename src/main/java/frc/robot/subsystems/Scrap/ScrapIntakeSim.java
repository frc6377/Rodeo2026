package frc.robot.subsystems.scrap;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.ArmConstants;
import frc.robot.Robot;

public class ScrapIntakeSim {
    
        // Simulation
    private SingleJointedArmSim armSim;
    private LoggedMechanism2d armMechanism2d;
    private LoggedMechanismRoot2d root;
    private LoggedMechanismLigament2d baseMech;
    private LoggedMechanismLigament2d scoringMech;
    
    public ScrapIntakeSim() {
                if (Robot.isSimulation()) {
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
            root = armMechanism2d.getRoot("Arm Sim", 0, 0)
            baseMech = root.append(new MechanismLigament2d(
                    "Arm Sim",
                    ArmConstants.kArmLength,
                    ArmConstants.kArmBaseAngle,
                    20,
                    new Color8Bit(Color.kBlue)));
            scoringMech = baseMech.append(new MechanismLigament2d(
                    "Scoring Elv", ArmConstants.kArmLength, 0, 10, new Color8Bit(Color.kAqua)));

            armTab.add("Arm Mech", armMechanism2d);
    }
}

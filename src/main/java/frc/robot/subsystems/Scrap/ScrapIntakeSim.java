package frc.robot.subsystems.scrap;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.ScrapArmConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ScrapIntakeSim extends ScrapIntakeReal {

    private SingleJointedArmSim armSim;
    private LoggedMechanism2d armMechanism2d;
    private LoggedMechanismRoot2d root;
    private LoggedMechanismLigament2d baseMech;

    private double armSetpoint = 0.0;

    public ScrapIntakeSim() {
        armSim = new SingleJointedArmSim(
                ScrapArmConstants.kArmMotor,
                ScrapArmConstants.kArmGearing,
                ScrapArmConstants.kArmMOI,
                ScrapArmConstants.kArmLength.in(Meters),
                ScrapArmConstants.kArmMinAngle.in(Radians),
                ScrapArmConstants.kArmMaxAngle.in(Radians),
                true,
                0);

        armMechanism2d = new LoggedMechanism2d(1, 1);
        root = armMechanism2d.getRoot("Arm Sim", 0, 0);
        baseMech = root.append(new LoggedMechanismLigament2d(
                "Arm Sim",
                ScrapArmConstants.kArmLength.in(Meters),
                ScrapArmConstants.kArmBaseAngle.in(Radians),
                20,
                new Color8Bit(Color.kBlue)));
        // Logger.recordOutput("Arm Mech", armMechanism2d);
    }

    @Override
    public void updateInputs(ScrapIntakeIOInputs inputs) {
        // Update simulation first
        armSim.setInput(pivotMotor.getMotorOutputPercent() * RobotController.getBatteryVoltage());
        armSim.update(0.02);

        // Update inputs from simulation
        inputs.armPositionDegrees = Radians.of(armSim.getAngleRads()).in(Degrees);
        inputs.armCurrentAmps = armSim.getCurrentDrawAmps();
        inputs.rollerCurrentAmps = intakeMotor.getStatorCurrent();
        inputs.atSetpoint = Math.abs(inputs.armPositionDegrees - super.armSetpoint) < 1.0;

        // Update visualization
        baseMech.setAngle(inputs.armPositionDegrees);
        Logger.recordOutput("Arm Mechanism", armMechanism2d);
    }
}

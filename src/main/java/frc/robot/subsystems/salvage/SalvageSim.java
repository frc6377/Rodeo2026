package frc.robot.subsystems.salvage;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.SalvageArmConstants;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class SalvageSim implements SalvageIO {
    private SingleJointedArmSim armSim;
    private LoggedMechanism2d armMechanism2d;
    private LoggedMechanismRoot2d root;
    private LoggedMechanismLigament2d baseMech;
    private final ShuffleboardTab salvageTab = Shuffleboard.getTab("Salvage");

    private double armSetpoint = 0.0;

    public SalvageSim() {
        armSim = new SingleJointedArmSim(
                SalvageArmConstants.kArmMotor,
                SalvageArmConstants.kArmGearing,
                SalvageArmConstants.kArmMOI,
                SalvageArmConstants.kArmLength.in(Meters),
                SalvageArmConstants.kArmMinAngle.in(Radians),
                SalvageArmConstants.kArmMaxAngle.in(Radians),
                true,
                SalvageArmConstants.kArmStartAngle.in(Radians));

        armMechanism2d = new LoggedMechanism2d(1, 1);
        root = armMechanism2d.getRoot("Salvage Arm Sim", 0, 0);
        baseMech = root.append(new LoggedMechanismLigament2d(
                "Salvage Arm Sim", SalvageArmConstants.kArmLength.in(Meters), 0, 20, new Color8Bit(Color.kGreen)));
        salvageTab.add("Salvage Arm Mech", armMechanism2d);
    }

    @Override
    public void updateInputs(SalvageIOInputs inputs) {
        // The voltage is controlled by the main Salvage subsystem's PID,
        // not by an internal sim PID. Just update the simulation.
        armSim.update(0.02);

        // Update mechanism display
        double currentAngle = Units.radiansToDegrees(armSim.getAngleRads());
        baseMech.setAngle(currentAngle);

        // Update telemetry
        inputs.armPositionDegrees = currentAngle;
        inputs.armVelocityDegreesPerSec = Units.radiansToDegrees(armSim.getVelocityRadPerSec());
        inputs.armCurrentAmps = armSim.getCurrentDrawAmps();
        inputs.intakeCurrentAmps = 2.0; // Simulated intake current
        inputs.atSetpoint = Math.abs(inputs.armPositionDegrees - armSetpoint) < SalvageArmConstants.PID.tolerance;

    }

    @Override
    public void setArmPosition(double degrees) {
        armSetpoint = degrees;
    }

    @Override
    public void setArmVoltage(double volts) {
        System.out.println("SalvageSim.setArmVoltage: " + volts + "V");
        armSim.setInput(volts);
    }

    @Override
    public void stopArm() {
        // Stop the arm motor but keep the current setpoint
        // This allows gravity compensation to continue working
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

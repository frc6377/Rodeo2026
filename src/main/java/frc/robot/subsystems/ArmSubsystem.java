package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.armConstants;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

public class ArmSubsystem extends SubsystemBase {
    // Add PIDController calculation to setArmMotor parameter
    private final VictorSPX m_armMotor;
    private final DutyCycleEncoder m_armEncoder;

    private static LoggedMechanism2d mech = new LoggedMechanism2d(2, 2);
    private DutyCycleEncoderSim m_armEncoderSim;
    private LoggedMechanismLigament2d armMech;
    private final PIDController armPID;
    private final ArmFeedforward armFeedforward;

    private SingleJointedArmSim m_armSim;

    public ArmSubsystem() {
        m_armMotor = new VictorSPX(8);
        m_armMotor.setNeutralMode(NeutralMode.Brake);
        m_armMotor.setInverted(InvertType.InvertMotorOutput);
        m_armEncoder = new DutyCycleEncoder(0, 1.0, .983);

        armPID = new PIDController(.02, 0, 0);
        armFeedforward = new ArmFeedforward(0, 0, 0, 0);

        if (Robot.isSimulation()) {
            m_armEncoderSim = new DutyCycleEncoderSim(m_armEncoder);

            m_armSim = new SingleJointedArmSim(
                    armConstants.kArmGearbox,
                    armConstants.kArmGearing,
                    SingleJointedArmSim.estimateMOI(
                            armConstants.armLength.in(Meters), Pounds.of(10).in(Kilograms)),
                    armConstants.armLength.in(Meters),
                    armConstants.armMinAngle.in(Radians),
                    armConstants.armMaxAngle.in(Radians),
                    true,
                    0);

            armMech = mech.getRoot("root", 1, 0)
                    .append(new LoggedMechanismLigament2d("Arm Mech [0]", 1, 0, 10, new Color8Bit(Color.kPurple)));
            Logger.recordOutput("Arm Mech", mech);
        }
    }

    public Command scoreSalvageCommand() {
        return setArmCommand(Degrees.of(40));
    }

    public Command floorPickupCommand() {
        return setArmCommand(Degrees.of(0.01));
    }

    public void setArmPercent(double percent) {
        m_armMotor.set(ControlMode.PercentOutput, percent);
    }

    public Angle getArmAngle() {
        return Robot.isSimulation() ? Radians.of(m_armEncoderSim.get()) : Rotations.of(m_armEncoder.get());
    }

    public Command setArmCommand(Angle target) {
        return Commands.sequence(
                        Commands.runOnce(() -> {
                            armPID.setSetpoint(target.in(Degrees));
                        }),
                        Commands.run(
                                () -> {
                                    double output =
                                            armPID.calculate(getArmAngle().in(Degrees));
                                    if (getArmAngle().in(Rotations) > .9) {
                                        output = armPID.calculate(getArmAngle().in(Degrees) - 360);
                                    } else {
                                        output = armPID.calculate(getArmAngle().in(Degrees));
                                    }
                                    double armFF = armFeedforward.calculate(target.in(Radians), 0);
                                    m_armMotor.set(ControlMode.PercentOutput, output + armFF);
                                    Logger.recordOutput("Arm/Output", output);
                                    Logger.recordOutput("Arm/Feed Forward", armFF);
                                },
                                this))
                .withName("Arm Go To command");
    }

    @Override
    public void periodic() {
        Logger.recordOutput(
                "Arm/Arm Subsystem command",
                getCurrentCommand() == null ? "null" : getCurrentCommand().getName());
        Logger.recordOutput("Arm/Motor Output", m_armMotor.getMotorOutputPercent());
        Logger.recordOutput("Arm/Motor Voltage", m_armMotor.getMotorOutputVoltage());
        Logger.recordOutput("Arm/Encoder Degrees", m_armEncoder.get());
    }

    public void simulationPeriodic() {
        m_armSim.setInputVoltage(m_armMotor.getMotorOutputVoltage());
        m_armSim.update(Robot.defaultPeriodSecs);
        final Angle simAngle = Radians.of(m_armSim.getAngleRads());
        m_armEncoderSim.set((simAngle).in(Rotations));
        Logger.recordOutput("Sim Angle in Radians", simAngle);
        Logger.recordOutput("Sim Angle in Degrees", simAngle.in(Degrees));
        armMech.setAngle(simAngle.in(Degrees));
        Logger.recordOutput("Arm Mech", mech);
    }
}

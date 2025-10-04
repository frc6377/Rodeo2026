package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.armConstants;
import frc.robot.Robot;

public class ArmSubsystem extends SubsystemBase {
    // Add PIDController calculation to setArmMotor parameter
    private final TalonSRX m_armMotor;
    private final DutyCycleEncoder m_armEncoder;

    private static Mechanism2d mech = new Mechanism2d(2, 2);
    private DutyCycleEncoderSim m_armEncoderSim;
    private MechanismLigament2d armMech;

    private SingleJointedArmSim m_armSim;

    public ArmSubsystem() {
        m_armMotor = new TalonSRX(6);
        m_armMotor.setNeutralMode(NeutralMode.Brake);
        m_armEncoder = new DutyCycleEncoder(1);

        if (Robot.isSimulation()) {
            m_armEncoderSim = new DutyCycleEncoderSim(m_armEncoder);

            m_armSim = new SingleJointedArmSim(
                    armConstants.kArmGearbox,
                    armConstants.kArmGearing,
                    0,
                    armConstants.armLength.in(Meters),
                    armConstants.armMinAngle.in(Radians),
                    armConstants.armMaxAngle.in(Radians),
                    true,
                    0,
                    null);

            armMech = mech.getRoot("root", 1, 0)
                    .append(new MechanismLigament2d("Arm Mech [0]", 1, 0, 10, new Color8Bit(Color.kPurple)));

            SmartDashboard.putData("Arm Mech", mech);
        }
    }

    public Command armDownCommand() {
        return startEnd(
                () -> {
                    double armPercent = armConstants.armPercent;

                    m_armMotor.set(ControlMode.PercentOutput, armPercent);
                },
                () -> m_armMotor.set(ControlMode.PercentOutput, 0));
    }

    public Command armUpCommand() {
        return startEnd(
                () -> {
                    double armPercent = -armConstants.armPercent;

                    m_armMotor.set(ControlMode.PercentOutput, armPercent);
                },
                () -> m_armMotor.set(ControlMode.PercentOutput, 0));
    }

    public void setArmPercent(double percent) {
        m_armMotor.set(ControlMode.PercentOutput, percent);
    }

    public Command setArmUpCommand(double angle, double percent) {
        return Commands.deadline(
                Commands.waitUntil(() -> m_armEncoder.get() >= angle),
                runEnd(
                        () -> {
                            setArmPercent(percent);
                        },
                        () -> {
                            setArmPercent(0);
                        }));
    }

    public Command setArmDownCommand(double angle, double percent) {
        return Commands.deadline(
                Commands.waitUntil(() -> m_armEncoder.get() <= angle),
                runEnd(
                        () -> {
                            setArmPercent(percent);
                        },
                        () -> {
                            setArmPercent(0);
                        }));
    }

    @Override
    public void simulationPeriodic() {
        m_armSim.setInputVoltage(m_armMotor.getMotorOutputVoltage());
        m_armSim.update(Robot.defaultPeriodSecs);
        final Angle simAngle = Radians.of(m_armSim.getAngleRads());
        m_armEncoderSim.set((simAngle).in(Rotations));

        armMech.setAngle(simAngle.in(Radians));
    }
}

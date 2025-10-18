package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.intakeConstants;
import frc.robot.Robot;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonSRX m_intakeMotor1;
    private final TalonSRX m_intakeMotor2;

    private FlywheelSim m_intakeSim;

    public IntakeSubsystem() {
        m_intakeMotor1 = new TalonSRX(5);
        m_intakeMotor2 = new TalonSRX(6);

        if (Robot.isSimulation()) {
            m_intakeSim = new FlywheelSim(
                    LinearSystemId.createFlywheelSystem(intakeConstants.kIntakeGearbox, 1, 1),
                    intakeConstants.kIntakeGearbox);
        }
    }

    public Command intakeCommand(DoubleSupplier leftTrigger, DoubleSupplier rightTrigger) {
        return run(() -> {
            double intakePercent =
                    -((leftTrigger.getAsDouble() - rightTrigger.getAsDouble()) * intakeConstants.intakePercent);
            m_intakeMotor1.set(ControlMode.PercentOutput, intakePercent);
            m_intakeMotor2.set(ControlMode.PercentOutput, -intakePercent);
        });
    }

    public void setIntakePercent(double percent) {
        m_intakeMotor1.set(ControlMode.PercentOutput, percent);
        m_intakeMotor2.set(ControlMode.PercentOutput, -percent);
    }

    public Command setIntakeCommand(double sec, double percent) {
        return Commands.deadline(
                Commands.waitSeconds(sec),
                runEnd(
                        () -> {
                            setIntakePercent(percent);
                        },
                        () -> {
                            setIntakePercent(0);
                        }));
    }

    @Override
    public void periodic() {
        Logger.recordOutput("intake/Motor 1 Output", m_intakeMotor1.getMotorOutputPercent());
        Logger.recordOutput("intake/Motor 2 Output", m_intakeMotor2.getMotorOutputPercent());
    }
}

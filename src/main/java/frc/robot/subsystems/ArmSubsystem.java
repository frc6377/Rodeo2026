package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.armConstants;

public class ArmSubsystem extends SubsystemBase {
  // Add PIDController calculation to setArmMotor parameter
  private final TalonSRX m_armMotor;
  private final CANcoder m_armEncoder;

  public ArmSubsystem() {
    m_armMotor = new TalonSRX(6);
    m_armMotor.setNeutralMode(NeutralMode.Brake);
    m_armEncoder = new CANcoder(1);
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
                Commands.waitUntil(() -> m_armEncoder.getPosition().getValueAsDouble() >= angle),
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
                Commands.waitUntil(() -> m_armEncoder.getPosition().getValueAsDouble() <= angle),
                runEnd(
                        () -> {
                            setArmPercent(percent);
                        },
                        () -> {
                            setArmPercent(0);
                        }));
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Salvage;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.salvageConstants;
import org.littletonrobotics.junction.Logger;

public class SalvageRoller extends SubsystemBase {
    /** Creates a new SalvageRoller. */
    private final TalonSRX salvageIntakeMotor;

    public SalvageRoller() {

        salvageIntakeMotor = new TalonSRX(Constants.MotorIDs.salvageIntakeMotor);
        salvageIntakeMotor.setInverted(false);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        Logger.recordOutput("Salvage/Intake Motor Output", salvageIntakeMotor.getMotorOutputPercent());
    }

    public Command intakeCommand() {
        return run(() -> salvageIntakeMotor.set(ControlMode.PercentOutput, salvageConstants.IntakeMotorSpeed))
                .finallyDo(() -> salvageIntakeMotor.set(ControlMode.PercentOutput, 0))
                .withName("intakeCommand");
    }

    public Command outtakeCommand() {
        return run(() -> salvageIntakeMotor.set(ControlMode.PercentOutput, -salvageConstants.IntakeMotorSpeed))
                .finallyDo(() -> salvageIntakeMotor.set(ControlMode.PercentOutput, 0))
                .withName("outtakeCommand");
    }

    public Command stopAll() {
        return Commands.runOnce(() -> {
            salvageIntakeMotor.set(ControlMode.PercentOutput, 0);
        });
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.scrapIntake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;

public class ScrapIntake extends SubsystemBase {
    private TalonSRX intakeMotor1;
    private TalonSRX intakeMotor2;

    /** Creates a new ScrapIntake. */
    public ScrapIntake() {
        intakeMotor1 = new TalonSRX(MotorIDs.scrapIntakeMotor1);
        intakeMotor2 = new TalonSRX(MotorIDs.scrapIntakeMotor2);
        intakeMotor2.follow(intakeMotor1);
        intakeMotor2.setInverted(true);
    }

    public Command intakeCommand() {
        return Commands.run(() -> {
            intakeMotor1.set(ControlMode.PercentOutput, 1);
        });
    }

    public Command outtakeCommand() {
        return Commands.run(() -> {
            intakeMotor1.set(ControlMode.PercentOutput, -1);
        });
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}

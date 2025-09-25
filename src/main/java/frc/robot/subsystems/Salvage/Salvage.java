// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of the
// WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Salvage;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.salvageConstants;
import frc.robot.Robot;

import org.littletonrobotics.junction.Logger;

public class Salvage extends SubsystemBase {
    /** Creates a new salvage. */

    // Motors
    private final TalonSRX salvagePivotLeader;

    private final TalonSRX salvagePivotFollower;

    private final TalonSRX salvageIntakeMotor;
    private final DutyCycleEncoder salvagePivotEncoder;

    // Sensors
    private final PIDController salvagePivotPID;

    public Salvage() {
        // Pivot Leader Motor
        salvagePivotLeader = new TalonSRX(Constants.MotorIDs.salvagePivotLeader);
        salvagePivotLeader.setInverted(false);

        // Pivot Follower Motor
        salvagePivotFollower = new TalonSRX(Constants.MotorIDs.salvagePivotFollower);
        salvagePivotFollower.setInverted(true);
        salvagePivotFollower.follow(salvagePivotLeader);

        // Intake Motor
        salvageIntakeMotor = new TalonSRX(Constants.MotorIDs.salvageIntakeMotor);
        salvageIntakeMotor.setInverted(false);

        // Sensors
        salvagePivotEncoder = new DutyCycleEncoder(Constants.SensorIDs.salvagePivotEncoder);
        salvagePivotPID = new PIDController(
                salvageConstants.salvagePivotP, salvageConstants.salvagePivotI, salvageConstants.salvagePivotD);
        salvagePivotPID.setTolerance(salvageConstants.SalvagePivotTolerance.in(Degrees));
    }

    public Command update() {
        return runOnce(() -> {
            if (Math.abs(salvagePivotPID.getSetpoint() - getCurrentAngle().in(Degrees))
                    > 180) { // Make sure the pivot does not go through the robot
                salvagePivotPID.setSetpoint(salvageConstants.SalvagePivotUpAngle.in(Degrees));
            }
            double output = salvagePivotPID.calculate(getCurrentAngle().in(Degrees));
            salvagePivotLeader.set(ControlMode.PercentOutput, output);

            // Logging
            Logger.recordOutput("Salvage/Pivot Setpoint", salvagePivotPID.getSetpoint());
            Logger.recordOutput("Salvage/Pivot Angle", getCurrentAngle().in(Degrees));
            Logger.recordOutput("Salvage/Pivot Leader Output", salvagePivotLeader.getMotorOutputPercent());
            Logger.recordOutput("Salvage/Pivot Follower Output", salvagePivotFollower.getMotorOutputPercent());
            Logger.recordOutput("Salvage/Pivot At Setpoint", salvagePivotPID.atSetpoint());
            Logger.recordOutput("Salvage/Intake Motor Output", salvageIntakeMotor.getMotorOutputPercent());
        });
    }

    public Angle getCurrentAngle() {
        return Degrees.of(salvagePivotEncoder.get() * 360);
    }

    public Command setAngle(Angle angle) {
        return Commands.runOnce(() -> {
            double clampedAngle = Math.max(
                    salvageConstants.SalvagePivotMinAngle.in(Degrees),
                    Math.min(salvageConstants.SalvagePivotMaxAngle.in(Degrees), angle.in(Degrees)));
            salvagePivotPID.setSetpoint(clampedAngle);
        });
    }

    public Command intakeCommand() {
        return run(() -> salvageIntakeMotor.set(ControlMode.PercentOutput, salvageConstants.IntakeMotorSpeed));
    }
    public Command holdCommand() {
        return run(() -> salvageIntakeMotor.set(ControlMode.PercentOutput, 0.1));
    }
    public Command outtakeCommand() {
        return run(() -> salvageIntakeMotor.set(ControlMode.PercentOutput, -salvageConstants.IntakeMotorSpeed));
    }

    public Command stopAll() {
        return Commands.runOnce(() -> {
            salvageIntakeMotor.set(ControlMode.PercentOutput, 0);
            salvagePivotLeader.set(ControlMode.PercentOutput, 0);
        });
    }

    public Command goToStowAngle() {
        return setAngle(salvageConstants.SalvagePivotStowAngle);
    }

    public Command goToPickupAngle() {
        return setAngle(salvageConstants.SalvagePivotPickupAngle);
    }

    public Command goToScoreAngle() {
        return setAngle(salvageConstants.SalvagePivotScoreAngle);
    }

    public Command intake() {
        return Commands.startEnd(
                () -> {
                    goToPickupAngle();
                    intakeCommand();
                },
                () -> {
                    goToStowAngle();
                    holdCommand();
                });
    }
    public Command outtake() {
        return Commands.startRun(
                () -> {
                    goToScoreAngle();
                    outtakeCommand();
                },
                () -> {
                    goToStowAngle();
                });
    }
    @Override
    public void periodic() {
        if (Robot.isReal()) {
            update().schedule();
        }

        //TODO: Don't move the logging, logging doesnt work when its inside the update command
        Logger.recordOutput("Salvage/Pivot Setpoint", salvagePivotPID.getSetpoint());
        Logger.recordOutput("Salvage/Pivot Angle", getCurrentAngle().in(Degrees));
        Logger.recordOutput("Salvage/Pivot Leader Output", salvagePivotLeader.getMotorOutputPercent());
        Logger.recordOutput("Salvage/Pivot Follower Output", salvagePivotFollower.getMotorOutputPercent());
        Logger.recordOutput("Salvage/Pivot At Setpoint", salvagePivotPID.atSetpoint());
        Logger.recordOutput("Salvage/Intake Motor Output", salvageIntakeMotor.getMotorOutputPercent());
    }
}

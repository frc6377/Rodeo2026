// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Salvage;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.salvageConstants;

public class Salvage extends SubsystemBase {
    /** Creates a new salvage. */

    // Motors
    private final TalonSRX salvagePivotMotor;

    private final TalonSRX salvageIntakeMotor;
    private final DutyCycleEncoder salvagePivotEncoder;
    private final PIDController salvagePivotPID;
    // Sensors
    public Salvage() {
        salvagePivotMotor = new TalonSRX(Constants.MotorIDs.salvagePivotMotor);
        salvagePivotMotor.setInverted(false);
        salvageIntakeMotor = new TalonSRX(Constants.MotorIDs.salvageIntakeMotor);
        salvageIntakeMotor.setInverted(false);
        salvagePivotEncoder = new DutyCycleEncoder(Constants.SensorIDs.salvagePivotEncoder);
        salvagePivotPID =  new PIDController(
            salvageConstants.salvagePivotP, salvageConstants.salvagePivotI, salvageConstants.salvagePivotD);
        salvagePivotPID.setTolerance(salvageConstants.SalvagePivotTolerance.in(Degrees));

    }

    public Command update() {
        return Commands.none();
    }


    // TODO make a stop command

    // TODO: Stop what? - ref. Line 47

    // TODO make a command to go to stow position

    public Angle getCurrentAngle() {
        return Degrees.of(salvagePivotEncoder.get()*360);
    }

    public Command setAngle(Angle angle) {
        return Commands.runOnce(() -> {
            double clampedAngle = Math.max(salvageConstants.SalvagePivotMinAngle.in(Degrees), Math.min(salvageConstants.SalvagePivotMaxAngle.in(Degrees), angle.in(Degrees)));
            salvagePivotPID.setSetpoint(clampedAngle);
        });
    }

    public Command intakeCommand(){
        return run(() -> salvageIntakeMotor.set(ControlMode.PercentOutput, salvageConstants.IntakeMotorSpeed));
    }

    public Command outtakeCommand(){
        return run(() -> salvageIntakeMotor.set(ControlMode.PercentOutput, -salvageConstants.IntakeMotorSpeed));
    }

    public Command stopAll(){
        return Commands.runOnce(() -> {
            salvageIntakeMotor.set(ControlMode.PercentOutput, 0);
            salvagePivotMotor.set(ControlMode.PercentOutput, 0);
        });
    }

    public Command goToStowAngle(){
        return setAngle(salvageConstants.SalvagePivotStowAngle);
    }

    public Command goToPickupAngle(){
        return setAngle(salvageConstants.SalvagePivotPickupAngle);
    }

    public Command goToScoreAngle(){
        return setAngle(salvageConstants.SalvagePivotScoreAngle);
    }


    @Override
    public void periodic() {
        if(!salvagePivotPID.atSetpoint()){
            setAngle(Degrees.of(salvagePivotPID.getSetpoint())).schedule();
        }
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of the
// WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Salvage;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.salvageConstants;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;

public class Salvage extends SubsystemBase {
    /** Creates a new salvage. */
    // Motors

    private final DutyCycleEncoder salvagePivotEncoder = new DutyCycleEncoder(Constants.SensorIDs.salvagePivotEncoder, 1, 0.6);


    private final SparkMax salvagePivotMotor = new SparkMax(Constants.MotorIDs.salvagePivotLeader, MotorType.kBrushed);
        // Pivot Leader Motor
    private final SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
            .withClosedLoopController(salvageConstants.salvagePivotP,salvageConstants.salvagePivotI, salvageConstants.salvagePivotD)
            .withSoftLimit(salvageConstants.SalvagePivotMinAngle, salvageConstants.SalvagePivotMaxAngle)
            .withExternalEncoder(salvagePivotEncoder)
            .withExternalEncoderGearing(45)
            .withGearing(45)
            .withTelemetry("Salvage/Pivot Motor",TelemetryVerbosity.HIGH)
            .withStatorCurrentLimit(Amps.of(40))
            .withFeedforward(new ArmFeedforward(0, 0, 0))
            .withClosedLoopTolerance(salvageConstants.SalvagePivotTolerance)
            .withControlMode(ControlMode.CLOSED_LOOP);
            
    private final SparkWrapper salvagePivotController = new SparkWrapper(salvagePivotMotor, DCMotor.getCIM(1), config);

    private ArmConfig salvagePivotArmConfig = new ArmConfig(salvagePivotController)
            .withLength(Inches.of(24))
            .withHardLimit(salvageConstants.SalvagePivotMinAngle, salvageConstants.SalvagePivotMaxAngle)
            .withTelemetry("Salvage/Pivot Arm", TelemetryVerbosity.HIGH)
            .withMass(Pounds.of(8))
            .withStartingPosition(Constants.salvageConstants.SalvagePivotInitialAngle);
    private final Arm salvageArm = new Arm(salvagePivotArmConfig);
    public Salvage() {

    }
    public Angle getCurrentAngle() {
        return Degrees.of(salvagePivotEncoder.get() * 360);
    }

    public Command setAngle(Angle angle) {
        return salvageArm.setAngle(angle);
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

    public Command goToInitialAngle() {
        return setAngle(salvageConstants.SalvagePivotInitialAngle);
    }

    @Override
    public void periodic() {
        salvageArm.updateTelemetry();
    }
}

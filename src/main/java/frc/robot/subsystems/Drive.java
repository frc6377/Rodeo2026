// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.MotorIDs;
import frc.robot.Robot;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
    private final TalonSRX leftDriveMotor1;
    private final TalonSRX leftDriveMotor2;
    private final TalonSRX rightDriveMotor1;
    private final TalonSRX rightDriveMotor2;

    private final Pigeon2 drivePigeon2;

    private Double targetAngle;

    // Simulation
    private DifferentialDrivetrainSim m_differentialDrivetrainSim;
    private Field2d m_field;

    /** Creates a new ExampleSubsystem. */
    public Drive() {
        leftDriveMotor1 = new TalonSRX(MotorIDs.leftDriveMotor1);
        leftDriveMotor1.setInverted(false);

        leftDriveMotor2 = new TalonSRX(MotorIDs.leftDriveMotor2);
        leftDriveMotor2.follow(leftDriveMotor1);
        leftDriveMotor2.setInverted(InvertType.OpposeMaster);

        rightDriveMotor1 = new TalonSRX(MotorIDs.rightDriveMotor1);
        rightDriveMotor1.setInverted(true);

        rightDriveMotor2 = new TalonSRX(MotorIDs.rightDriveMotor2);
        rightDriveMotor2.follow(rightDriveMotor1);
        rightDriveMotor2.setInverted(InvertType.OpposeMaster);

        drivePigeon2 = new Pigeon2(MotorIDs.pigeonID);
        drivePigeon2.setYaw(0);

        targetAngle = drivePigeon2.getYaw().getValueAsDouble();

        if (Robot.isSimulation()) {
            m_field = new Field2d();
            SmartDashboard.putData("Field", m_field);

            m_differentialDrivetrainSim = DifferentialDrivetrainSim.createKitbotSim(
                    KitbotMotor.kDualCIMPerSide, // 2 CIMs per side.
                    KitbotGearing.k10p71, // 10.71:1
                    KitbotWheelSize.kSixInch, // 6" diameter wheels.
                    null // No measurement noise.
                    );
            m_differentialDrivetrainSim.setPose(new Pose2d(0.0, 4.5, new Rotation2d()));
        }
    }

    // Getters
    public Trigger isGyroInRange(double target) {
        return new Trigger(() -> targetAngle - DriveConstants.angleTolerance < getDriveAngleDeg()
                && getDriveAngleDeg() < targetAngle + DriveConstants.angleTolerance);
    }

    // Functions
    public void setLeftPercent(double percent) {
        leftDriveMotor1.set(ControlMode.PercentOutput, percent);
    }

    public void setRightPercent(double percent) {
        rightDriveMotor1.set(ControlMode.PercentOutput, percent);
    }

    public double getDriveAngleDeg() {
        if (Robot.isSimulation()) {
            return m_differentialDrivetrainSim.getPose().getRotation().getDegrees();
        } else {
            return drivePigeon2.getYaw().getValueAsDouble();
        }
    }

    // Commands
    public Command driveCommand(DoubleSupplier forwardAxis, DoubleSupplier turnAxis) {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return run(() -> {
            double leftPercent = (forwardAxis.getAsDouble() * DriveConstants.maxDrivePercent)
                    + (turnAxis.getAsDouble() * DriveConstants.maxTurnPercent);
            double rightPercent = (forwardAxis.getAsDouble() * DriveConstants.maxDrivePercent)
                    + (-turnAxis.getAsDouble() * DriveConstants.maxDrivePercent);

            leftDriveMotor1.set(ControlMode.PercentOutput, leftPercent);
            rightDriveMotor1.set(ControlMode.PercentOutput, rightPercent);
        });
    }

    // Auto Functions
    public Command setForwardCommand(double sec, double percent) {
        return Commands.deadline(
                Commands.waitSeconds(sec),
                runEnd(
                        () -> {
                            setLeftPercent(percent);
                            setRightPercent(percent);
                        },
                        () -> {
                            setLeftPercent(0);
                            setRightPercent(0);
                        }));
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Drive/Left Motor 1", leftDriveMotor1.getMotorOutputPercent());
        Logger.recordOutput("Drive/Left Motor 2", leftDriveMotor2.getMotorOutputPercent());
        Logger.recordOutput("Drive/Right Motor 1", rightDriveMotor1.getMotorOutputPercent());
        Logger.recordOutput("Drive/Right Motor 2", rightDriveMotor2.getMotorOutputPercent());

        Logger.recordOutput("Drive/Pigeon Yaw", getDriveAngleDeg());
        Logger.recordOutput("Drive/Target Angle Auto", targetAngle);
        SmartDashboard.putString(
                "DriveCommand",
                getCurrentCommand() != null ? getCurrentCommand().getName() : "No Command");
    }

    @Override
    public void simulationPeriodic() {
        m_differentialDrivetrainSim.setInputs(
                leftDriveMotor1.getMotorOutputPercent() * RobotController.getBatteryVoltage(),
                rightDriveMotor1.getMotorOutputPercent() * RobotController.getBatteryVoltage());
        m_differentialDrivetrainSim.update(0.02);

        m_field.setRobotPose(m_differentialDrivetrainSim.getPose());
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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
import frc.robot.Constants.SensorIDs;
import frc.robot.Robot;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
    private final TalonSRX leftDriveLeader;
    private final VictorSPX leftDriveFollower;
    private final TalonSRX rightDriveLeader;
    private final VictorSPX rightDriveFollower;

    private final Pigeon2 gyro;

    private final DifferentialDrive diffDrive;

    private Double targetAngle;

    private Pigeon2SimState gyroSim = null;

    // Simulation
    private DifferentialDrivetrainSim diffDriveSim;
    private Field2d m_field;

    /** Creates a new ExampleSubsystem. */
    public Drive() {
        leftDriveLeader = new TalonSRX(MotorIDs.leftDriveLeader);
        leftDriveLeader.setInverted(false);
        leftDriveLeader.setNeutralMode(NeutralMode.Brake);

        leftDriveFollower = new VictorSPX(MotorIDs.leftDriveFollower);
        leftDriveFollower.follow(leftDriveLeader);
        leftDriveFollower.setInverted(InvertType.FollowMaster);
        leftDriveFollower.setNeutralMode(NeutralMode.Brake);

        rightDriveLeader = new TalonSRX(MotorIDs.rightDriveLeader);
        rightDriveLeader.setInverted(true);
        rightDriveLeader.setNeutralMode(NeutralMode.Brake);

        rightDriveFollower = new VictorSPX(MotorIDs.rightDriveFollower);
        rightDriveFollower.follow(rightDriveLeader);
        rightDriveFollower.setInverted(InvertType.FollowMaster);
        rightDriveFollower.setNeutralMode(NeutralMode.Coast);

        gyro = new Pigeon2(SensorIDs.pigeonID);
        gyro.setYaw(0);

        diffDrive = new DifferentialDrive(
                (speed) -> leftDriveLeader.set(ControlMode.PercentOutput, speed),
                (speed) -> rightDriveLeader.set(ControlMode.PercentOutput, speed));

        targetAngle = gyro.getYaw().getValueAsDouble();

        if (Robot.isSimulation()) {
            m_field = new Field2d();
            gyroSim = new Pigeon2SimState(gyro);
            SmartDashboard.putData("Field", m_field);

            diffDriveSim = DifferentialDrivetrainSim.createKitbotSim(
                    KitbotMotor.kDualCIMPerSide, // 2 CIMs per side.
                    KitbotGearing.k7p31, // 10.71:1
                    KitbotWheelSize.kSixInch, // 6" diameter wheels.
                    null // No measurement noise.
                    );
            diffDriveSim.setPose(new Pose2d(0.0, 4.5, new Rotation2d()));
        }
    }

    // Getters
    public Trigger isGyroInRange(double target) {
        return new Trigger(() -> targetAngle - DriveConstants.angleTolerance < getDriveAngleDeg()
                && getDriveAngleDeg() < targetAngle + DriveConstants.angleTolerance);
    }

    // Functions
    public void setLeftPercent(double percent) {
        leftDriveLeader.set(ControlMode.PercentOutput, percent);
    }

    public void setRightPercent(double percent) {
        rightDriveLeader.set(ControlMode.PercentOutput, percent);
    }

    public double getDriveAngleDeg() {
        if (Robot.isSimulation()) {
            return diffDriveSim.getPose().getRotation().getDegrees();
        } else {
            return gyro.getYaw().getValueAsDouble();
        }
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        diffDrive.arcadeDrive(
                speeds.vxMetersPerSecond / DriveConstants.maxSpeed.in(MetersPerSecond) / (2 * Math.PI),
                speeds.omegaRadiansPerSecond / DriveConstants.maxRotation.in(RadiansPerSecond));
    }

    // Commands
    public Command arcadeDrive(DoubleSupplier forward, DoubleSupplier rotation) {
        return run(() -> diffDrive.arcadeDrive(forward.getAsDouble(), rotation.getAsDouble()));
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

        Logger.recordOutput("Drive/Left Motor 1", leftDriveLeader.getMotorOutputPercent());
        Logger.recordOutput("Drive/Left Motor 2", leftDriveFollower.getMotorOutputPercent());
        Logger.recordOutput("Drive/Right Motor 1", rightDriveLeader.getMotorOutputPercent());
        Logger.recordOutput("Drive/Right Motor 2", rightDriveFollower.getMotorOutputPercent());

        Logger.recordOutput("Drive/LeftOutput", leftDriveLeader.getMotorOutputPercent());
        Logger.recordOutput("Drive/RightOutput", rightDriveLeader.getMotorOutputPercent());

        Logger.recordOutput("Drive/Target Angle Auto", targetAngle);
        SmartDashboard.putString(
                "DriveCommand",
                getCurrentCommand() != null ? getCurrentCommand().getName() : "No Command");

        if (Robot.isReal()) {
            Logger.recordOutput("Drive/Pigeon Yaw", getDriveAngleDeg());
        }
    }

    @Override
    public void simulationPeriodic() {
        diffDriveSim.setInputs(
                leftDriveLeader.getMotorOutputPercent() * RobotController.getBatteryVoltage(),
                rightDriveLeader.getMotorOutputPercent() * RobotController.getBatteryVoltage());

        diffDriveSim.update(0.02);

        m_field.setRobotPose(diffDriveSim.getPose());

        Logger.recordOutput("Drive/SIM - Robot Pose", m_field.getRobotPose());
        Logger.recordOutput(
                "Drive/SIM - Robot Rotation", diffDriveSim.getHeading().getDegrees());

        gyroSim.setRawYaw(diffDriveSim.getHeading().getDegrees());
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import static edu.wpi.first.units.Units.KilogramMetersSquaredPerSecond;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
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

    private final DifferentialDriveOdometry diffOdometry;

    private final DifferentialDriveKinematics kinematics;

    private DifferentialDriveWheelSpeeds wheelSpeeds;

    private final RobotConfig robotConfig;
    private final ModuleConfig driveModuleConfig;

    private final Encoder leftEncoder;
    private final Encoder rightEncoder;

    // Simulation
    private DifferentialDrivetrainSim m_differentialDrivetrainSim;
    private Field2d m_field;

    /** Creates a new ExampleSubsystem. */
    public Drive() {
        leftDriveLeader = new TalonSRX(MotorIDs.leftDriveLeader);
        leftDriveLeader.setInverted(false);

        leftDriveFollower = new VictorSPX(MotorIDs.leftDriveFollower);
        leftDriveFollower.follow(leftDriveLeader);
        leftDriveFollower.setInverted(InvertType.FollowMaster);

        rightDriveLeader = new TalonSRX(MotorIDs.rightDriveLeader);
        rightDriveLeader.setInverted(true);

        rightDriveFollower = new VictorSPX(MotorIDs.rightDriveFollower);
        rightDriveFollower.follow(rightDriveLeader);
        rightDriveFollower.setInverted(InvertType.FollowMaster);

        gyro = new Pigeon2(SensorIDs.pigeonID);
        gyro.setYaw(0);

        leftEncoder = new Encoder(SensorIDs.driveLeftEncoderA, SensorIDs.driveLeftEncoderB, true);
        leftEncoder.setDistancePerPulse(
                DriveConstants.wheelDiameter.in(Meters) * Math.PI / DriveConstants.encoderResolution);

        rightEncoder = new Encoder(SensorIDs.driveRightEncoderA, SensorIDs.driveRightEncoderB, false);
        leftEncoder.setDistancePerPulse(
                DriveConstants.wheelDiameter.in(Meters) * Math.PI / DriveConstants.encoderResolution);

        diffDrive = new DifferentialDrive(
                (speed) -> leftDriveLeader.set(ControlMode.PercentOutput, speed),
                (speed) -> leftDriveLeader.set(ControlMode.PercentOutput, speed));

        diffOdometry = new DifferentialDriveOdometry(
                gyro.getRotation2d(),
                leftEncoder.getDistance(),
                rightEncoder.getDistance(),
                new Pose2d(0.0, 0.0, new Rotation2d(0.0)));

        kinematics = new DifferentialDriveKinematics(DriveConstants.trackWidth.in(Meters));
        wheelSpeeds = new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());

        targetAngle = gyro.getYaw().getValueAsDouble();

        driveModuleConfig = new ModuleConfig(
                DriveConstants.wheelDiameter.div(2.0),
                DriveConstants.maxSpeed,
                DriveConstants.wheelCOF,
                DCMotor.getCIM(2),
                DriveConstants.motorCurrentLimit,
                2);
        robotConfig = new RobotConfig(
                DriveConstants.robotMass.in(Kilograms),
                DriveConstants.robotMOI.in(KilogramMetersSquaredPerSecond),
                null,
                DriveConstants.trackWidth.in(Meters));

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

        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting
                // pose)
                this::getCurrentSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds) -> driveRobotRelative(
                        speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds.
                new PPLTVController(0.02), // PPLTVController is the built in path following controller for differential
                // drive trains
                robotConfig,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
                );
    }

    // Getters
    public Trigger isGyroInRange(double target) {
        return new Trigger(() -> targetAngle - DriveConstants.angleTolerance < getDriveAngleDeg()
                && getDriveAngleDeg() < targetAngle + DriveConstants.angleTolerance);
    }

    public Pose2d getPose() {
        return diffOdometry.getPoseMeters();
    }

    public ChassisSpeeds getCurrentSpeeds() {
        return kinematics.toChassisSpeeds(wheelSpeeds);
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
            return m_differentialDrivetrainSim.getPose().getRotation().getDegrees();
        } else {
            return gyro.getYaw().getValueAsDouble();
        }
    }

    public void resetOdometry(Pose2d resetPose) {
        diffOdometry.resetPose(resetPose);
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        diffDrive.arcadeDrive(
                speeds.vxMetersPerSecond / DriveConstants.maxSpeed.in(MetersPerSecond),
                speeds.vxMetersPerSecond / DriveConstants.maxSpeed.in(MetersPerSecond));
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
        diffOdometry.update(gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
        wheelSpeeds = new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());

        Logger.recordOutput("Drive/Left Motor 1", leftDriveLeader.getMotorOutputPercent());
        Logger.recordOutput("Drive/Left Motor 2", leftDriveFollower.getMotorOutputPercent());
        Logger.recordOutput("Drive/Right Motor 1", rightDriveLeader.getMotorOutputPercent());
        Logger.recordOutput("Drive/Right Motor 2", rightDriveFollower.getMotorOutputPercent());

        Logger.recordOutput("Drive/Pigeon Yaw", getDriveAngleDeg());
        Logger.recordOutput("Drive/Target Angle Auto", targetAngle);
        SmartDashboard.putString(
                "DriveCommand",
                getCurrentCommand() != null ? getCurrentCommand().getName() : "No Command");
    }

    @Override
    public void simulationPeriodic() {
        m_differentialDrivetrainSim.setInputs(
                leftDriveLeader.getMotorOutputPercent() * RobotController.getBatteryVoltage(),
                rightDriveLeader.getMotorOutputPercent() * RobotController.getBatteryVoltage());
        m_differentialDrivetrainSim.update(0.02);

        m_field.setRobotPose(m_differentialDrivetrainSim.getPose());
    }
}

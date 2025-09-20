// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Pivot extends SubsystemBase {
    public TalonSRX pivotMotor;
    public CANcoder encoder;

    // Sim
    private SingleJointedArmSim pivotSim;
    private LoggedMechanism2d pivotMech2d;
    private LoggedMechanismRoot2d pivotRootMech;
    private LoggedMechanismLigament2d pivotMechLigament;

    /** Creates a new Pivot. */
    public Pivot() {
        pivotMotor = new TalonSRX(MotorIDs.pivotMotor);
        pivotMotor.config_kP(0, 1);
        pivotMotor.config_kI(0, 0);
        pivotMotor.config_kD(0, 0);
        pivotMotor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0, 0, 10);
        pivotMotor.configRemoteFeedbackFilter(MotorIDs.pivotEncoder, RemoteSensorSource.CANCoder, 0, 10);

        encoder = new CANcoder(MotorIDs.pivotEncoder);

        if (Robot.isSimulation()) {
            pivotSim = new SingleJointedArmSim(
                    PivotConstants.kGearbox,
                    PivotConstants.kGearing,
                    PivotConstants.kMOI,
                    PivotConstants.kArmLength.in(Meters),
                    PivotConstants.kMinAngle.in(Radians),
                    PivotConstants.kMaxAngle.in(Radians),
                    PivotConstants.kSimulateGravity,
                    PivotConstants.kStartAngle.in(Radians));
            pivotMech2d = new LoggedMechanism2d(1, 1);
            pivotRootMech = pivotMech2d.getRoot("Root", 0.5, 0.5);
            pivotMechLigament =
                    pivotRootMech.append(new LoggedMechanismLigament2d("Pivot", 0.5, 0, 10, new Color8Bit(Color.kRed)));
        }
    }

    public Command stowedPoseCommand() {
        return Commands.runOnce(() -> {
            pivotMotor.set(ControlMode.Position, PivotConstants.kStowedPose.in(Degrees));
        });
    }

    public Command scorePoseCommand() {
        return Commands.runOnce(() -> {
            pivotMotor.set(ControlMode.Position, PivotConstants.kScoredPose.in(Degrees));
        });
    }

    public Command testCommand() {
        return Commands.run(() -> {
            pivotMotor.set(ControlMode.PercentOutput, 1);
        });
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        Logger.recordOutput("Pivot/Motor Percent", pivotMotor.getMotorOutputPercent());
        Logger.recordOutput(
                "Pivot/Encoder Angle Deg", encoder.getPosition().getValue().in(Degrees));
    }

    @Override
    public void simulationPeriodic() {
        pivotSim.setInput(pivotMotor.getMotorOutputPercent() * RobotController.getBatteryVoltage());
        pivotSim.update(Robot.defaultPeriodSecs);

        pivotMechLigament.setAngle(Units.radiansToDegrees(pivotSim.getAngleRads()));

        encoder.setPosition(Radians.of(pivotSim.getAngleRads()));

        Logger.recordOutput("Pivot/Sim Angle Deg", Units.radiansToDegrees(pivotSim.getAngleRads()));
        Logger.recordOutput("Pivot/Mech2d", pivotMech2d);
    }
}

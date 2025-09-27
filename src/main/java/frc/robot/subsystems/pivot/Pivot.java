// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DIOConstants;
import frc.robot.Constants.MotorIDs;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Pivot extends SubsystemBase {
    private TalonSRX pivotMotor;

    // Sim
    private SingleJointedArmSim pivotSim;
    private LoggedMechanism2d pivotMech2d;
    private LoggedMechanismRoot2d pivotRootMech;
    private LoggedMechanismLigament2d pivotMechLigament;

    private final DutyCycleEncoder encoder;
    private DutyCycleEncoderSim encoderSim;
    private final PIDController pivotPID;
    private final ArmFeedforward pivotFeedforward;

    // PID
    private LoggedNetworkNumber tuneP;
    private LoggedNetworkNumber tuneI;
    private LoggedNetworkNumber tuneD;

    /** Creates a new Pivot. */
    public Pivot() {
        pivotMotor = new TalonSRX(MotorIDs.pivotMotor);

        encoder = new DutyCycleEncoder(DIOConstants.pivotEncoderID);

        pivotPID = new PIDController(1, 0, 0);
        pivotFeedforward = new ArmFeedforward(0, 0, 0, 0);

        tuneP = new LoggedNetworkNumber("Pivot/PID/P");
        tuneI = new LoggedNetworkNumber("Pivot/PID/I");
        tuneD = new LoggedNetworkNumber("Pivot/PID/D");

        if (Robot.isSimulation()) {
            encoderSim = new DutyCycleEncoderSim(encoder);

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

    public Angle getPivotAngle() {
        return Robot.isSimulation()
                ? Radians.of(encoderSim.get())
                : Radians.of(encoder.get() + PivotConstants.kOffset.in(Radians));
    }

    public Command pivotToPose(Angle targetPose) {
        return Commands.sequence(
                        Commands.runOnce(() -> {
                            pivotPID.setSetpoint(targetPose.in(Degrees));
                        }),
                        Commands.run(
                                () -> {
                                    double output =
                                            pivotPID.calculate(getPivotAngle().in(Degrees));
                                    pivotMotor.set(ControlMode.PercentOutput, output);
                                    Logger.recordOutput("Pivot/Output (Deg)", output);
                                },
                                this))
                .withName("pivotToPose");
    }

    public Command stowedPoseCommand() {
        return pivotToPose(PivotConstants.kStowedPose).withName("stowedPoseCommand");
    }

    public Command scorePoseCommand() {
        return pivotToPose(PivotConstants.kScoredPose).withName("scorePoseCommand");
    }

    public Command testCommand() {
        return Commands.run(
                        () -> {
                            pivotMotor.set(ControlMode.PercentOutput, 1);
                        },
                        this)
                .withName("Test");
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        Logger.recordOutput("Pivot/Motor Percent", pivotMotor.getMotorOutputPercent());
        Logger.recordOutput("Pivot/Encoder Angle Deg", getPivotAngle().in(Degrees));
        Logger.recordOutput(
                "Pivot/PivotCommand",
                getCurrentCommand() != null ? getCurrentCommand().getName() : "No Command");
        Logger.recordOutput("Pivot/Set Point (Deg)", pivotPID.getSetpoint());

        tuneP.periodic();
        tuneI.periodic();
        tuneD.periodic();

        pivotPID.setPID(tuneP.get(), tuneI.get(), tuneD.get());

        Logger.recordOutput("Pivot/PID/P", pivotPID.getP());
        Logger.recordOutput("Pivot/PID/I", pivotPID.getI());
        Logger.recordOutput("Pivot/PID/D", pivotPID.getD());

        Logger.recordOutput("Pivot/PID/tuneP", tuneP.get());
    }

    @Override
    public void simulationPeriodic() {
        pivotSim.setInput(pivotMotor.getMotorOutputPercent() * RobotController.getBatteryVoltage());
        pivotSim.update(Robot.defaultPeriodSecs);

        pivotMechLigament.setAngle(Units.radiansToDegrees(pivotSim.getAngleRads()));

        encoderSim.set(pivotSim.getAngleRads());

        Logger.recordOutput("Pivot/Sim Angle Deg", Units.radiansToDegrees(pivotSim.getAngleRads()));
        Logger.recordOutput("Pivot/Mech2d", pivotMech2d);
        Logger.recordOutput(
                "Pivot/Sim Encoder Pos (Deg)", Radians.of(encoderSim.get()).in(Degrees));
    }
}

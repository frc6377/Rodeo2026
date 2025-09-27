package frc.robot.subsystems.salvage;

import static edu.wpi.first.units.Units.Degrees;

import java.util.Set;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.SensorIDs;
import frc.robot.Constants.PivotConstants;


public class Salvage extends SubsystemBase {
    private TalonSRX intakeMotor;
    private TalonSRX armMotor;
    private final DutyCycleEncoder salvagePivotEncoder;

    public Salvage() {
        intakeMotor = new TalonSRX(MotorIDs.salvageMotor);
        armMotor = new TalonSRX(MotorIDs.salvageArmMotor);
        salvagePivotEncoder = new DutyCycleEncoder(SensorIDs.salvagePivotEncoder);
    }

    public Angle getCurrentAngle() {
        return Degrees.of(salvagePivotEncoder.get() * 360);
    }

    public Command intakeCommand() {
        return Commands.run(() -> {
            intakeMotor.set(ControlMode.PercentOutput, 1);
        });
    }

    public Command outtakeCommand() {
        return Commands.run(() -> {
            intakeMotor.set(ControlMode.PercentOutput, -1);
        });
    }

    // 4 setpoints: intake, stow, low, high stow TO DO SATURDAY

    public enum Setpoint {
        INTAKE(Degrees.of(0)),
        STOW(Degrees.of(45)),
        FREIGHT(Degrees.of(90)),
        HIGHSTOW(Degrees.of(135));

        private final Angle angle;

        Setpoint(Angle angle) {
            this.angle = angle;
        }

        public Angle getAngle() {
            return angle;
        }
    }

    public Command moveArmCommand(Setpoint setpoint) {
        return Commands.run(() -> {
            armMotor.set(ControlMode.Position, setpoint.getAngle().in(Degrees)*PivotConstants.gearRatio);
        });
    }

    @Override
    public void periodic() {

    }
}
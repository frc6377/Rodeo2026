package frc.robot.autos;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Salvage.Salvage;
import frc.robot.subsystems.Salvage.SalvageRoller;

public final class Autos {

    // Example autonomous routine: Drive forward and then turn
    public static Command driveAndTurn(Drive drivetrain, Time driveTime, Time turnTime) {
        return AutoCommands.driveForward(drivetrain, driveTime).withName("Drive");
    }

    // Example autonomous routine: Pick up salvage and score it
    public static Command pickupAndScore(Salvage salvage, SalvageRoller roller, Time rollerTime) {
        return Commands.sequence(
                        AutoCommands.intakeSalvageArm(salvage),
                        AutoCommands.setSalvageRoller(roller, rollerTime),
                        AutoCommands.outakeSalvageArm(salvage))
                .withName("PickupAndScore");
    }

    public static Command oneSalvageMiddle(
            Salvage salvage, SalvageRoller roller, Drive drivetrain, Time driveTime, Time rollerTime) {
        return Commands.sequence(
                        AutoCommands.driveForward(drivetrain, driveTime),
                        AutoCommands.outakeSalvageArm(salvage),
                        AutoCommands.setSalvageRoller(roller, rollerTime))
                .withName("OneSalvageMiddle");
    }

    public static Command driveAndTurn(Drive drivetrain) {
        return driveAndTurn(drivetrain, Seconds.of(3), Seconds.of(1.5));
    }

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}

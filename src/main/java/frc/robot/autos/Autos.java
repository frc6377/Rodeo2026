package frc.robot.autos;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Salvage.Salvage;

public final class Autos {

    // Example autonomous routine: Drive forward and then turn
    public static Command driveAndTurn(Drive drivetrain, Time driveTime, Time turnTime) {
        return Commands.sequence(
                        AutoCommands.driveForward(drivetrain, driveTime), AutoCommands.driveTurn(drivetrain, turnTime))
                .withName("DriveAndTurn");
    }

    // Example autonomous routine: Pick up salvage and score it
    public static Command pickupAndScore(Salvage salvage, Time rollerTime) {
        return Commands.sequence(
                        AutoCommands.intakeSalvageArm(salvage),
                        AutoCommands.setSalvageRoller(salvage, rollerTime),
                        AutoCommands.outakeSalvageArm(salvage))
                .withName("PickupAndScore");
    }

    public static Command oneSalvageMiddle(Salvage salvage, Drive drivetrain, Time driveTime, Time rollerTime) {
        return Commands.sequence(
                        AutoCommands.driveForward(drivetrain, driveTime),
                        AutoCommands.outakeSalvageArm(salvage),
                        AutoCommands.setSalvageRoller(salvage, rollerTime))
                .withName("OneSalvageMiddle");
    }

    public static Command driveAndTurn(Drive drivetrain) {
        return driveAndTurn(drivetrain, Seconds.of(2), Seconds.of(1.5));
    }

    public static Command pickupAndScore(Salvage salvage) {
        return pickupAndScore(salvage, Seconds.of(1));
    }

    public static Command oneSalvageMiddle(Salvage salvage, Drive drivetrain) {
        return oneSalvageMiddle(salvage, drivetrain, Seconds.of(8), Seconds.of(5));
    }

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}

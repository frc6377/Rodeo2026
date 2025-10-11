// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Salvage.Salvage;

public final class AutoCommands {
    /** Example static factory for an autonomous command. */
    public static Command driveForward(Drive drivetrain, Time time) {
        return Commands.sequence(drivetrain.arcadeDrive(() -> 0.75, () -> 0))
                .withTimeout(time.in(Seconds))
                .withName("driveForward");
    }

    public static Command driveTurn(Drive drivetrain, Time time) {
        return Commands.sequence(drivetrain.arcadeDrive(() -> 0, () -> 0.75))
                .withTimeout(time.in(Seconds))
                .withName("driveTurn");
    }

    public static Command setSalvageArmANgle(Salvage arm, Angle angle) {
        return arm.setAngle(angle);
    }

    public static Command setSalvageRoller(Salvage roller, Time time) {
        return roller.intakeCommand().withTimeout(time.in(Seconds));
    }

    public static Command intakeSalvageArm(Salvage arm) {
        return arm.goToPickupAngle();
    }

    public static Command outakeSalvageArm(Salvage arm) {
        return arm.goToScoreAngle();
    }

    private AutoCommands() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}

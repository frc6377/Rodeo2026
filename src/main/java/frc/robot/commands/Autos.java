// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.salvage.*;
import frc.robot.subsystems.scrap.*;

public final class Autos {
    // Go forward (1.5s) turn 45deg go forward (2s) turn -45deg go forward (1s) score salvage
    public static Command LeftScoreAuto(
            Drive driveSubsystem,
            Salvage salvageSubsystem,
            ScrapIntake scrapIntakeSubsystem,
            ScrapShooter scrapShooterSubsystem) {
        int turnDegrees = 30;
        double scaler = 1;
        return Commands.sequence(
                        driveSubsystem.setForwardCommand(2.5 * scaler, 0.45),
                        driveSubsystem.turnCommand(turnDegrees).withTimeout(2),
                        driveSubsystem.setForwardCommand(4 * scaler, 0.45),
                        driveSubsystem.turnCommand(-turnDegrees).withTimeout(2),
                        driveSubsystem.setForwardCommand(2 * scaler, 0.45),
                        salvageSubsystem.moveArmCommand(Salvage.Setpoint.FREIGHT),
                        salvageSubsystem.holdArmPositionCommand(),
                        salvageSubsystem.outtakeCommand())
                .withName("LeftScoreAuto");
    }

    // ONLY GETS 0.5s TO SCORE
    public static Command CenterScoreAuto(
            Drive driveSubsystem,
            Salvage salvageSubsystem,
            ScrapIntake scrapIntakeSubsystem,
            ScrapShooter scrapShooterSubsystem) {
        int turnDegrees = 30;
        double scaler = 1;
        return Commands.sequence(
                        driveSubsystem.setForwardCommand(1.5 * scaler, 0.45),
                        driveSubsystem.turnCommand(-15).withTimeout(2),
                        driveSubsystem.setForwardCommand(2 * scaler, 0.45),
                        driveSubsystem.turnCommand(turnDegrees + 15).withTimeout(2),
                        driveSubsystem.setForwardCommand(4 * scaler, 0.45),
                        driveSubsystem.turnCommand(-turnDegrees).withTimeout(2),
                        driveSubsystem.setForwardCommand(1 * scaler, 0.45),
                        salvageSubsystem.moveArmCommand(Salvage.Setpoint.FREIGHT),
                        salvageSubsystem.holdArmPositionCommand(),
                        salvageSubsystem.outtakeCommand())
                .withName("CenterScoreAuto");
    }

    public static Command LeaveDockAuto(
            Drive driveSubsystem,
            Salvage salvageSubsystem,
            ScrapIntake scrapIntakeSubsystem,
            ScrapShooter scrapShooterSubsystem) {
        return Commands.sequence(driveSubsystem.setForwardCommand(1.5, 0.45)).withName("LeaveDockAuto");
    }

    public static Command ShootScrapAuto(
            Drive driveSubsystem,
            Salvage salvageSubsystem,
            ScrapIntake scrapIntakeSubsystem,
            ScrapShooter scrapShooterSubsystem) {
        return Commands.sequence(
                        driveSubsystem.setForwardCommand(1.5, 0.45),
                        driveSubsystem.turnCommand(225).withTimeout(2),
                        driveSubsystem.setForwardCommand(1.75, -0.45),
                        scrapIntakeSubsystem.intake().withTimeout(2),
                        Commands.waitSeconds(1),
                        driveSubsystem.turnCommand(225).withTimeout(2),
                        scrapShooterSubsystem.startShooter(1),
                        Commands.waitSeconds(5),
                        scrapShooterSubsystem.stopShooter())
                .withName("ShootScrapAuto");
    }
    // Commands
    // Note: forwards and backwards are reversed for the drive base due to how the Xbox controller
    // works
    // So negative numbers will be forwards, and positive will be backwards.
    public static Command AutoTurn90Test(Drive driveTrainSubsystem) {
        return Commands.sequence(driveTrainSubsystem.turnCommand(90), driveTrainSubsystem.turnCommand(-90))
                .withName("AutoTurn90Test");
    }

    public static Command AutoForwardTest(Drive driveTrainSubsystem) {
        return Commands.sequence(
                        driveTrainSubsystem.setForwardCommand(0.75, -0.5),
                        Commands.waitSeconds(2),
                        driveTrainSubsystem.setForwardCommand(0.75, 0.5))
                .withName("AutoForwardTest");
    }

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}

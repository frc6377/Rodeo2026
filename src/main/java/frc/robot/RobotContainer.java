// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Salvage.Salvage;
import frc.robot.util.OILayer.OI;
import frc.robot.util.OILayer.OIKeyboard;
import frc.robot.util.OILayer.OIXbox;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer {
    // Subsystems
    private final Drive drive;
    private final Salvage salvage;

    // Controller
    private final OI controller =
            Constants.currentMode.equals(Constants.Mode.SIM) && Constants.useKeyboard ? new OIKeyboard() : new OIXbox();

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive = new Drive();
                salvage = new Salvage();
                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                drive = new Drive();
                salvage = new Salvage();
                break;

            default:
                // Replayed robot, disable IO implementations
                drive = new Drive();
                salvage = new Salvage();
                break;
        }

        // Set up auto routines (Not AutoBuilder.buildAutoChooser() - Tank Don't Have Odometry)
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", new SendableChooser<Command>());

        // Set up SysId routines
        autoChooser.addOption("Example Auto", Commands.none());

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(drive.driveCommand(controller.driveTranslationY(), controller.driveRotation()));
        // Reset gyro / odometry
        final Runnable resetGyro = () -> {};
        controller.zeroDrivebase().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

        // Salvage Control
        controller.rightTriggerButton()
                .whileTrue(salvage.intake());
        controller.rightBumper()
                .whileTrue(salvage.outtake());
        
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    // *** Normally Used for MapleSim ***
    public void resetSimulationField() {
        if (Constants.currentMode != Constants.Mode.SIM) return;
    }

    // *** Normally Used for MapleSim ***
    public void updateSimulation() {
        if (Constants.currentMode != Constants.Mode.SIM) return;
    }
}

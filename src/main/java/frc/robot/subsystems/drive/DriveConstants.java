// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

/** Add your docs here. */
public class DriveConstants {
    // Sum of these value should be <= 1 (can be over, would not cause problems)
    public static final double kDriveP = 0.0075;
    public static final double kDriveI = 0.0;
    public static final double kDriveD = 0.0;

    public static final double minTurnSpeed = 0.1;

    public static final double maxDrivePercent = 0.5;
    public static final double maxTurnPercent = 0.5;

    public static final double angleTolerance = 2;

    public static final double minPower = 0.3;
    public static final double debounce = 1;
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramMetersSquaredPerSecond;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.AngularMomentum;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;

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

    public static final double encoderResolution = 2048;
    public static final Distance wheelCircumference = Inches.of(6).times(Math.PI);
    public static final Distance wheelDiameter = Inches.of(6);

    public static final Distance trackWidth = Inches.of(18);

    public static final LinearVelocity maxSpeed = MetersPerSecond.of(4.0); // TODO:FIXIT
    public static final AngularVelocity maxRotation = RadiansPerSecond.of(6.5); // TODO:FIXIT

    public static final Mass robotMass = Kilograms.of(50); // TODO:FIXIT

    public static final AngularMomentum robotMOI = KilogramMetersSquaredPerSecond.of(6.0); // TODO:FIXIT

    public static final double wheelCOF = 0.095;

    public static final Current motorCurrentLimit = Amps.of(40);
}

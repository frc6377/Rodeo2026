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

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running on a roboRIO. Change
 * the value of "simMode" to switch between "sim" (physics sim) and "replay" (log replay from a file).
 */
public final class Constants {
    public static final boolean tuningMode = false;
    public static final boolean useKeyboard = true;
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public final class MotorIDs {
        public static final int leftDriveMotor1 = 2;
        public static final int leftDriveMotor2 = 1;
        public static final int rightDriveMotor1 = 4;
        public static final int rightDriveMotor2 = 3;

        public static final int pigeonID = 10;
    }

    public final class armConstants {
        public static final double armPercent = .225;
        public static final double kArmGearing = 1;

        // Simulation Constants
        public static final DCMotor kArmGearbox = DCMotor.getMiniCIM(1);
        public static final Distance armLength = Inches.of(20);
        public static final Mass k = Pounds.of(4.75);
        public static final Angle armMinAngle = Degrees.of(0);
        public static final Angle armMaxAngle = Degrees.of(45);
    }

    public static class intakeConstants {
        public static final double intakePercent = .225;

        // Simulation Constants
        public static final DCMotor kIntakeGearbox = DCMotor.getMiniCIM(1);
    }
}

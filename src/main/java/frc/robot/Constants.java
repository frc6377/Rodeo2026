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

        // Drive Motors
        public static final int leftDriveMotor1 = 3;
        public static final int leftDriveMotor2 = 4;
        public static final int rightDriveMotor1 = 1;
        public static final int rightDriveMotor2 = 2;

        // Pigeon
        public static final int pigeonID = 5;

        // Scrap Motors IDs
        public static final int intakeMotorID = 6;
        public static final int pivotMotorID = 7;
        public static final int pivotEncoderID = 9;
        public static final int shooterMotor1ID = 10;
        public static final int shooterMotor2ID = 11;

        // Salvage Motor IDs
        public static final int salvagePivotMotorID = 8;
    }
}

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

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
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

    public final class ScrapArmConstants {
        // Simulation constants
        public static final DCMotor kArmMotor = DCMotor.getCIM(2);
        public static final double kArmGearing = 40;
        public static final double kArmMOI = 4.625509184446; // Units = m^2 kg
        public static final double kArmMinAngle = Units.degreesToRadians(0);
        public static final double kArmMaxAngle = Units.degreesToRadians(90);
        public static final double kArmLength = Units.inchesToMeters(25.88);
        public static final double kArmBaseAngle = 63.6;
        public static final double kArmScoringAngle = Units.degreesToRadians(-45);
    }

    public final class MotorIDs {

        // Drive Motors
        public static final int leftDriveMotor1 = 4;
        public static final int leftDriveMotor2 = 3;
        public static final int rightDriveMotor1 = 2;
        public static final int rightDriveMotor2 = 1;

        // Pigeon
        public static final int pigeonID = 9;

        // Scrap Motors IDs
        public static final int intakeMotorID = 6;
        public static final int pivotMotorID = 7;
        public static final int pivotEncoderID = 50;
        public static final int shooterMotor1ID = 10;
        public static final int shooterMotor2ID = 11;

        // Salvage Motor IDs
        public static final int salvageMotor = 8;
        public static final int salvageArmMotor = 12;
    }

    public final class SensorIDs {

        public static final int salvagePivotEncoder = 4;
    }

    public final class PivotConstants {
        public static final double gearRatio = 40; // TO BE CHANGED
    }
}

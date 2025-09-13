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

import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.HowdyPID;

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
        public static final int leftDriveLeader = 1;
        public static final int leftDriveFollower = 2;
        public static final int rightDriveLeader = 3;
        public static final int rightDriveFollower = 4;

        public static final int pigeonID = 5;
        
        public static final int salvagePivotMotor = 6;
        public static final int salvageIntakeMotor = 7;
    }
    public final class SensorIDs {
        public static final int salvagePivotEncoder = 0; // TODO: FIXIT

    }
    public static class salvageConstants {
        //PID
        public static final double salvagePivotP = 1.0; // TODO: FIXIT
        public static final double salvagePivotI = 0.0;
        public static final double salvagePivotD = 0.0;
        //Angles
        public static final Angle SalvagePivotMinAngle = Degrees.of(0); // TODO: FIXIT
        public static final Angle SalvagePivotMaxAngle = Degrees.of(0); // degrees
        public static final Angle SalvagePivotTolerance = Degrees.of(2); // degrees
        public static final Angle SalvagePivotInitialAngle = Degrees.of(0); // degrees
        public static final Angle SalvagePivotPickupAngle = Degrees.of(0); // degrees
        public static final Angle SalvagePivotStowAngle = Degrees.of(0); // degrees
        public static final Angle SalvagePivotScoreAngle = Degrees.of(0); // degrees
        public static final Angle SalvagePivotUpAngle = Degrees.of(0); // degrees
        //Speeds
        public static final double IntakeMotorSpeed = 0.7;
        public static final double OuttakeMotorSpeed = -0.7;
        
    }
}

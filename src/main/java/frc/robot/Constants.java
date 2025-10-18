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
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running on a roboRIO. Change
 * the value of "simMode" to switch between "sim" (physics sim) and "replay" (log replay from a file).
 */
public final class Constants {
    public static final boolean isJared = false;

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
        public static final Angle kArmMinAngle = Degrees.of(0); // 0 degrees
        public static final Angle kArmMaxAngle = Degrees.of(90); // 90 degrees
        public static final Distance kArmLength = Inches.of(25.88);
        public static final Angle kArmBaseAngle = Degrees.of(0);
        public static final Angle kArmScoringAngle = Degrees.of(-45); // -45 degrees
        public static final Angle kArmIntakeAngle = Degrees.of(-5); // -45 degrees
        public static final Angle kArmStowAngle = Degrees.of(90); // -45 degrees
        public static final double kArmMOI = SingleJointedArmSim.estimateMOI(
                kArmLength.in(Meters), Pounds.of(10).in(Kilograms));

        public static class PID {
            public static final double kP = 0.1;
            public static final double kI = 0.0;
            public static final double kD = 0.01;
        }

        public static class FEEDFORWARD {
            public static final double kS = 0.1;
            public static final double kG = 0.1;
            public static final double kV = 0.1;
            public static final double kA = 0.1;
        }
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
        public static final int pivotEncoderID = 12;
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

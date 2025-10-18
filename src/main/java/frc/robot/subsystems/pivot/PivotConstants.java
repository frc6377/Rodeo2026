// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Add your docs here. */
public class PivotConstants {
    public static final Angle kStowedPose = Degrees.of(10);
    public static final Angle kScoredPose = Degrees.of(45);

    // 2 foot long arm, 1 cim motor, start angle = 0, end angle = 90, arm mass = 7.5
    public static final DCMotor kGearbox = DCMotor.getCIM(1);

    // Mech Sim
    public static final double kGearing = 25;
    public static final Distance kArmLength = Inches.of(24);
    public static final Angle kMinAngle = Degrees.of(0);
    public static final Angle kMaxAngle = Degrees.of(180);
    public static final boolean kSimulateGravity = true;
    public static final Angle kStartAngle = Degrees.of(0);
    public static final double kMOI = SingleJointedArmSim.estimateMOI(kArmLength.in(Meters), Units.lbsToKilograms(7.5));

    public static final Angle kOffset = Degrees.of(-79);
}

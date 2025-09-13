// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Salvage;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.salvageConstants;

public class Salvage extends SubsystemBase {
  /** Creates a new salvage. */

  //Motors
  private final TalonSRX salvagePivotMotor;
  private final TalonSRX salvageIntakeMotor;
  private final DutyCycleEncoder salvagePivotEncoder;
  private final PIDController salvagePivotPID = new PIDController(salvageConstants.salvagePivotP, salvageConstants.salvagePivotI, salvageConstants.salvagePivotD);
  // Sensors
  public Salvage() {
    salvagePivotMotor = new TalonSRX(Constants.MotorIDs.salvagePivotMotor);
    salvagePivotMotor.setInverted(false);
    salvageIntakeMotor = new TalonSRX(Constants.MotorIDs.salvageIntakeMotor);
    salvageIntakeMotor.setInverted(false);
    salvagePivotEncoder = new DutyCycleEncoder(Constants.SensorIDs.salvagePivotEncoder);
  

  }
  public Command update() {

    return Commands.none();
  }
  // TODO make a go to angle command
  // TODO make an intake command
  // TODO make an outtake command
  // TODO make a stop command
  // TODO make a command to go to stow position

  public double getCurrentAngle(){
    return 0.0; // TODO: FIXIT
  }

  @Override
  public void periodic() {
    
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants;
import frc.robot.Constants.HandConstants;

public class HandSubsystem extends SubsystemBase {
  /** Creates a new Hand Subsystem. */

  private CANSparkMax handMotor;
  public HandSubsystem() {

    handMotor = new CANSparkMax(HandConstants.kHandID, com.revrobotics.CANSparkLowLevel.MotorType.kBrushed);
    handMotor.setSmartCurrentLimit(HandConstants.kHandCurrentLimit);
  }
  
  public Command getIntakeCommand() {
    return this.startEnd(
      () -> {
        setHandMotor(HandConstants.kIntakePower);
      },
      () -> {
        stop();
      }
    );
  }

  public Command getOuttakeCommand() {
    return this.startEnd(
      () -> {
        setHandMotor(HandConstants.kOuttakePower);
      },
      () -> {
        stop();
      }
    );
  }
  public void setHandMotor(double power) {
    handMotor.set(power);
  }
  public void stop() {
    handMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AmpTrapConstants;
import frc.robot.Constants.HandConstants;

public class AmpTrapSubsystem extends SubsystemBase {

  private CANSparkMax ampMotor;

  /** Creates a new AmpTrapSubsystem. */
  public AmpTrapSubsystem() {
    ampMotor = new CANSparkMax(Constants.AmpTrapConstants.kAmpId, CANSparkLowLevel.MotorType.kBrushed);

  }

  public void setAmpMotor(double power) {
    ampMotor.set(power);
  }

  public Command getIntakeCommand() {
    return this.startEnd(
      () -> {
        setAmpMotor(AmpTrapConstants.kIntakePower);
      },
      () -> {
        stop();
      }
    );
  }

  public Command getOuttakeCommand() {
    return this.startEnd(
      () -> {
        setAmpMotor(AmpTrapConstants.kOuttakePower);
      },
      () -> {
        stop();
      }
    );
  }

  public void stop() {
    ampMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

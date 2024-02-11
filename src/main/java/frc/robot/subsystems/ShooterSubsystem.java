// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.LauncherConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

  private CANSparkMax topMotor;
  private CANSparkMax lowerMotor;
  public ShooterSubsystem() {
    topMotor = new CANSparkMax(LauncherConstants.kLauncherID, MotorType.kBrushed);
    lowerMotor = new CANSparkMax(LauncherConstants.kFeederID, MotorType.kBrushed);
    topMotor.setSmartCurrentLimit(LauncherConstants.kLauncherCurrentLimit);
    lowerMotor.setSmartCurrentLimit(LauncherConstants.kFeedCurrentLimit);
  }

  public Command getIntakeCommand() {
    return this.
    startEnd(() -> {setTopMotor(LauncherConstants.kIntakeTopMotor);
       setLowerMotor(LauncherConstants.kIntakeLowerMotor);} ,
     ()->{stop();});
  }

  public Command getAmpOuttakeCommand() {
    return this.
    startEnd(() -> {
      setTopMotor(LauncherConstants.kAmpTopMotorSpeed);
      setLowerMotor(LauncherConstants.kAmpLowerMotorSpeed);
    },
    () -> {
      stop();
    });
  }
  public void setTopMotor(double power) {
    topMotor.set(-power);
  }

  public void setLowerMotor(double power) {
    lowerMotor.set(-power);
  }
  public void stop() {
    topMotor.set(0);
    lowerMotor.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

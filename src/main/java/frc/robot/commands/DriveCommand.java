//Copy and pasted from Yzma

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends Command {

  // Field for DriveSubsystem
  private final DriveSubsystem m_robotDrive;

  private double speedControl = 0.5;
  private double rateLimit = 1.5;

  private SlewRateLimiter filterX = new SlewRateLimiter(rateLimit);
  private SlewRateLimiter filterY = new SlewRateLimiter(rateLimit);

  // Limelight Fields
  private NetworkTable limTable;
  private NetworkTableEntry tx;
  private NetworkTableEntry ledMode;

  // PID Control for Limelight Turn
  private double kP = .0375;//was .025

  // For PID Control
  double turnPower = 0;




private boolean fieldCentric = true;


  /** Creates a new DriveCommand. */
  public DriveCommand(DriveSubsystem drive) {

    // Limelight Initialization
    limTable = NetworkTableInstance.getDefault().getTable("limelight");
    tx = limTable.getEntry("tx");
    ledMode = limTable.getEntry("ledMode");


    

    // Use addRequirements() here to declare subsystem dependencies.
    m_robotDrive = drive;
    addRequirements(m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ledMode.setDouble(1); // 3 is on, 1 is off
    tx = limTable.getEntry("tx");
  }

  double leftX = 0;
  double leftY = 0;
  double rightX = 0;


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Updated Drive Command

    // Use these for Competition
    //leftX = filterX.calculate(Robot.getDriveControlJoystick().getRawAxis(0)) * speedControl;
    //leftY = filterY.calculate(Robot.getDriveControlJoystick().getRawAxis(1)) * speedControl;

    // Slow Mode for Testing
    leftX = Robot.getDriveControlJoystick().getRawAxis(0) * speedControl;
    leftY = Robot.getDriveControlJoystick().getRawAxis(1) * speedControl;

    // Leave this as is
    rightX = Robot.getDriveControlJoystick().getRawAxis(4); 

    // Clip to Max Value
    

    if (Robot.getDriveControlJoystick().getRawAxis(2) > 0.5) { //left trigger
      ledMode.setDouble(3); // On
      double error = tx.getDouble(0);
      turnPower = kP * error;
      leftX = turnPower;
      if(fieldCentric = true) {
        leftY *= -1;
      }
      leftY=-0.5;
      System.out.println(tx.getDouble(0) + ", " + turnPower);
      fieldCentric=false;
      
    }
    else {
      ledMode.setDouble(1); // Off
      turnPower = 0;
      fieldCentric=true;
    }


    // Rate Limiting is currently ENABLED
    m_robotDrive.drive(
                MathUtil.applyDeadband(-leftY*speedControl, 0.06),
                MathUtil.applyDeadband(-leftX*speedControl, 0.06),
                MathUtil.applyDeadband(-rightX*speedControl, 0.06),
                fieldCentric,
                true);

                

    //TODO: deal with rotation offset (reference 2024 alpha)
    if(Robot.getDriveControlJoystick().getPOV()!=-1){ // Left Stick Button
      
     m_robotDrive.zeroHeading(Robot.getDriveControlJoystick().getPOV());
     //m_robotDrive.zeroHeading();
    }

    // Speed Control
    // Left Bumper Full Speed
    if (Robot.getDriveControlJoystick().getRawButton(5)) {
      speedControl = 1.0;
      rateLimit = 10;
    }
    // Right Bumper Half Speed
    if (Robot.getDriveControlJoystick().getRawButton(6)) {
      speedControl = 0.5;
      rateLimit = 1.5;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Todo: Set motors to stop

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
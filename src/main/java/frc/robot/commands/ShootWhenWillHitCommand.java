// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.LimelightConstants;
public class ShootWhenWillHitCommand extends Command {
    ShooterSubsystem m_ShooterSubsystem;
    NetworkTable limTable;
    private NetworkTableEntry ty;
    private NetworkTableEntry tx;
    private NetworkTableEntry ledMode;
  /** Creates a new ShooterCommand. */
  public ShootWhenWillHitCommand(ShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ShooterSubsystem = shooter;
    addRequirements(m_ShooterSubsystem);

    // Limelight Initialization
    limTable = NetworkTableInstance.getDefault().getTable("limelight");
    tx = limTable.getEntry("tx");
    ledMode = limTable.getEntry("ledMode");
    ty = limTable.getEntry("ty");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ShooterSubsystem.setTopMotor(LauncherConstants.kLauncherSpeed);
  }

  double xError;
  double yError;
  int timer;
  boolean isFound = false;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(tx.exists()&&ty.exists()&&!isFound) {
        xError = tx.getDouble(0)-LimelightConstants.speakerAimtx;
      yError = LimelightConstants.speakerAimty-ty.getDouble(0);

      if(xError < 3 && yError <3) {
        isFound = true;
        m_ShooterSubsystem.setLowerMotor(LauncherConstants.kLaunchFeederSpeed);
      }
    }
    if(isFound) timer++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShooterSubsystem.stop();
    timer = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer > 60;
  }
}

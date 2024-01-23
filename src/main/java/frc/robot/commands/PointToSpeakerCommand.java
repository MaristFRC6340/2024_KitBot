// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.LimelightConstants;
public class PointToSpeakerCommand extends Command {

// Limelight Fields
  private NetworkTable limTable;
  private NetworkTableEntry tx;
  private NetworkTableEntry ledMode;
  private NetworkTableEntry ty;

    private DriveSubsystem m_DriveSubsystem;
  /** Creates a new ShooterCommand. */
  public PointToSpeakerCommand(DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_DriveSubsystem = drive;
    addRequirements(m_DriveSubsystem);

    // Limelight Initialization
    limTable = NetworkTableInstance.getDefault().getTable("limelight");
    tx = limTable.getEntry("tx");
    ledMode = limTable.getEntry("ledMode");
    ty = limTable.getEntry("ty");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ledMode.setDouble(3); // 3 is on, 1 is off
    tx = limTable.getEntry("tx");
    ty = limTable.getEntry("ty");
  }

  double rotSpeed;

  double xError;
  double yError;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(tx.getDouble(0)!=0&&ty.getDouble(0)!=0)
    {
      xError = tx.getDouble(0)-LimelightConstants.speakerAimtx;
      yError = LimelightConstants.speakerAimty-ty.getDouble(0);

      System.out.print("x: " + xError);
      System.out.println("y: " + yError);
      m_DriveSubsystem.drive(
          yError*LimelightConstants.kPY,
          xError*LimelightConstants.kPX*-1,
          0,
          false,
          true
      );
    }
    else {
      m_DriveSubsystem.drive(
          yError*LimelightConstants.kPY,
          xError*LimelightConstants.kPX*-1,
          0,
          false,
          true
      );
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ledMode.setDouble(1);
    m_DriveSubsystem.drive(0, 0, 0, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if((Math.abs(xError)<LimelightConstants.tolerance && Math.abs(yError)<LimelightConstants.tolerance) && tx.getDouble(0)!=0) {
        return true;
    }
    return false;
  }
}

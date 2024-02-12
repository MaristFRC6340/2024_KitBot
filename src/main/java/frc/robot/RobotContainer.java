// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveToAmpCommand;
import frc.robot.commands.LaunchNote;
import frc.robot.commands.PointToSpeakerCommand;
import frc.robot.commands.PrepareLaunch;
import frc.robot.commands.ShootWhenWillHitCommand;
import frc.robot.commands.StopShooter;
import frc.robot.subsystems.AmpTrapSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TelescopingAmpTestSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  //private final HandSubsystem m_HandSubsystem = new HandSubsystem();
  //private final AmpTrapSubsystem m_ArmTrapSubsystem = new AmpTrapSubsystem();
  private final TelescopingAmpTestSubsystem m_TelescopingAmpTestSubsystem = new TelescopingAmpTestSubsystem();
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_actuatorController = new XboxController(OIConstants.kActuatorControllerPort);

  //Create Triggers
  Trigger y = new JoystickButton(m_driverController, XboxController.Button.kY.value);
  Trigger a = new JoystickButton(m_driverController, XboxController.Button.kA.value);
  Trigger b = new JoystickButton(m_driverController, XboxController.Button.kB.value);
  Trigger x = new JoystickButton(m_driverController, XboxController.Button.kX.value);
  Trigger rBumper = new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);
  Trigger lBumper = new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);

  Trigger actuatorA = new JoystickButton(m_actuatorController, XboxController.Button.kA.value);
  Trigger actuatorB = new JoystickButton(m_actuatorController, XboxController.Button.kB.value);
  Trigger actuatorX = new JoystickButton(m_actuatorController, XboxController.Button.kX.value);
  Trigger actuatorY = new JoystickButton(m_actuatorController, XboxController.Button.kY.value);
  
  Trigger actuatorLeftBumper = new JoystickButton(m_actuatorController, XboxController.Button.kLeftBumper.value);
  Trigger actuatorRightBumper = new JoystickButton(m_actuatorController, XboxController.Button.kRightBumper.value);

  Trigger actuatorRightTrigger = new Trigger(() -> m_actuatorController.getRightTriggerAxis()>.2);
  Trigger actuatorLeftTrigger = new Trigger(() -> m_actuatorController.getLeftTriggerAxis()>.2);

  Trigger actuatorLeftY = new Trigger(() -> Math.abs(m_actuatorController.getLeftY())>.1);
  Trigger actuatorRightY = new Trigger(() -> Math.abs(m_actuatorController.getRightY())>.1);
  //Trigger leftShouler = new JoystickButton(m_driverController, XboxController.Button.k.value);
  private final SendableChooser<Command> autoChooser;

  private final DriveCommand m_DriveCommand = new DriveCommand(m_robotDrive);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    //Register Named Commands for auto
    configureNamedCommands();
    // Configure the button bindings
    configureButtonBindings();
    // Configure default commands
    // m_robotDrive.setDefaultCommand(
    //     // The left stick controls translation of the robot.
    //     // Turning is controlled by the X axis of the right stick.
    //     new RunCommand(
    //         () -> m_robotDrive.drive(
    //             -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
    //             -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
    //             -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
    //             true, true),
    //         m_robotDrive));
    m_robotDrive.setDefaultCommand(m_DriveCommand);

    //Create sendable chooser and give it to the smartdashboard
    autoChooser = AutoBuilder.buildAutoChooser("Example Auto2");
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }


  /**
   * Register all NamedCommands here for use in pathplanner
   * general style is camelcase for NamedCommands to differentiate them from the class name
   */
  private void configureNamedCommands() {
    NamedCommands.registerCommand("prepareLaunch", new PrepareLaunch(m_ShooterSubsystem).withTimeout(.5));
    NamedCommands.registerCommand("launchNote", new LaunchNote(m_ShooterSubsystem).withTimeout(.5));
    NamedCommands.registerCommand("intakeSource", m_ShooterSubsystem.getIntakeCommand());
    NamedCommands.registerCommand("stopShooter", new StopShooter(m_ShooterSubsystem));
    NamedCommands.registerCommand("aimAndScore", new SequentialCommandGroup(new PointToSpeakerCommand(m_robotDrive),
       new PrepareLaunch(m_ShooterSubsystem).withTimeout(.5).handleInterrupt(() -> m_ShooterSubsystem.stop()),
        new WaitCommand(.25),
        new LaunchNote(m_ShooterSubsystem)
        ));
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    
    y.whileTrue(
        new PrepareLaunch(m_ShooterSubsystem)
        .withTimeout(.25)
        .andThen(new LaunchNote(m_ShooterSubsystem))
        .handleInterrupt(()-> m_ShooterSubsystem.stop())
    );

    a.whileTrue(
        m_ShooterSubsystem.getIntakeCommand()
    );

    
    // x.whileTrue(
    //   m_HandSubsystem.getOuttakeCommand()
    // );
    

    // lBumper.whileTrue(
    //   m_HandSubsystem.getIntakeCommand()
    // );

    // // AmpTrap Event Listeners
    // actuatorY.whileTrue(m_ArmTrapSubsystem.getOuttakeCommand());

    // actuatorA.whileTrue(m_ArmTrapSubsystem.getIntakeCommand());


     
    b.whileTrue(
      new SequentialCommandGroup(new PointToSpeakerCommand(m_robotDrive),
       new PrepareLaunch(m_ShooterSubsystem).withTimeout(.5),
        new WaitCommand(.25),
        new LaunchNote(m_ShooterSubsystem)).handleInterrupt(() -> m_ShooterSubsystem.stop())
    );
    x.whileTrue(new SequentialCommandGroup(
      new DriveToAmpCommand(m_robotDrive),
      m_robotDrive.getDriveForwardCommand(.2).withTimeout(1),
      m_robotDrive.getDriveForwardCommand(-.1).withTimeout(.23),
      m_TelescopingAmpTestSubsystem.getRotateNoteCommand(-.2).withTimeout(1),
      m_TelescopingAmpTestSubsystem.getOuttakeNoteCommand().withTimeout(2)
    ));

    actuatorLeftBumper.whileTrue(m_TelescopingAmpTestSubsystem.getMoveTelescopeCommand(-.2));
    actuatorRightBumper.whileTrue(m_TelescopingAmpTestSubsystem.getMoveTelescopeCommand(.2));
    actuatorLeftTrigger.whileTrue(m_TelescopingAmpTestSubsystem.getRotateNoteCommand(.2));
    actuatorRightTrigger.whileTrue(m_TelescopingAmpTestSubsystem.getRotateNoteCommand(-.2));

      actuatorLeftY.whileTrue(m_TelescopingAmpTestSubsystem.getRunLowerFingerCommand(() -> m_actuatorController.getLeftY()));
      actuatorRightY.whileTrue(m_TelescopingAmpTestSubsystem.getRunTopFingerCommand(() -> m_actuatorController.getRightY()));

    actuatorA.whileTrue(m_TelescopingAmpTestSubsystem.getIntakeNoteCommand());
    actuatorY.whileTrue(m_TelescopingAmpTestSubsystem.getOuttakeNoteCommand());
    
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    

    // // Create config for trajectory
    // TrajectoryConfig config = new TrajectoryConfig(
    //     AutoConstants.kMaxSpeedMetersPerSecond,
    //     AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //     // Add kinematics to ensure max speed is actually obeyed
    //     .setKinematics(DriveConstants.kDriveKinematics);

    // // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(3, 0, new Rotation2d(0)),
    //     config);

    // var thetaController = new ProfiledPIDController(
    //     AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    //     exampleTrajectory,
    //     m_robotDrive::getPose, // Functional interface to feed supplier
    //     DriveConstants.kDriveKinematics,

    //     // Position controllers
    //     new PIDController(AutoConstants.kPXController, 0, 0),
    //     new PIDController(AutoConstants.kPYController, 0, 0),
    //     thetaController,
    //     m_robotDrive::setModuleStates,
    //     m_robotDrive);

    // // Reset odometry to the starting pose of the trajectory.
    // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // // Run path following command, then stop at the end.
    // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));

    return autoChooser.getSelected();

    //return this.getOnTheFlyPath();
  }

  public Command getDriveCommand() {
    return m_DriveCommand;
  }

  public Command getOnTheFlyPath() {
    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
      new Pose2d(1, 1, Rotation2d.fromDegrees(0)),
      new Pose2d(2, 1, Rotation2d.fromDegrees(0))

    );

    PathPlannerPath path = new PathPlannerPath(
      bezierPoints,
      new PathConstraints(3, 3, 6.28, 12.56),
      new GoalEndState(0, Rotation2d.fromDegrees(-90))
    );

    path.preventFlipping=true;

    return AutoBuilder.followPath(path);
  }
  

  //AUTO COMMANDS


  // public Command getExamplePathAutoOnTheFly() {
  //     List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
  //       new Pose2d(1, 1, Rotation2d.fromDegrees(0)),
  //       new Pose2d(3, 1, Rotation2d.fromDegrees(90)),
  //       new Pose2d(5, 1, Rotation2d.fromDegrees(180))
  //     );

  //     PathPlannerPath path =  new PathPlannerPath(bezierPoints, new PathConstraints(3, 3, 2*Math.PI, 4*Math.PI), new GoalEndState(0, Rotation2d.fromDegrees(-90)));

  //     return AutoBuilder.followPathWithEvents(path);
  // }

  // public Command getExampleAutoCommand() {
  //   return new PathPlannerAuto("Example Auto");
  // }


}

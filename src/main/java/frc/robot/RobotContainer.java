// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.IntakingCommand;
import frc.robot.commands.ResetClimbCommand;
import frc.robot.commands.ScoringCommand;
import frc.robot.commands.TimedIntakeSetPowerCommand;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Scoring;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  public final Scoring score = new Scoring();
  //public final Limelight servo = new Limelight(score);
  public final Climb climbSub = new Climb();

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driverController = new CommandXboxController(0); // all joysticks initially negative (removed)
  CommandXboxController armController = new CommandXboxController(1);
  CommandXboxController testingController = new CommandXboxController(2);

  SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));

  private void configureBindings() {

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverController.getRightX());
    
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    driverController.y().onTrue(new InstantCommand(() -> drivebase.zeroGyro()));

    driverController.povDown().onTrue(new InstantCommand(() -> 
      score.ext.setTargetInches(score.ext.getTargetInches() - 2)));

    driverController.povUp().onTrue(new InstantCommand(() -> 
      score.ext.setTargetInches(score.ext.getTargetInches() + 2)));
 
    // ARM
    armController.leftTrigger().onTrue(new ScoringCommand(score, Constants.ScoringPos.AMP));

    // /* hold */
    armController.rightTrigger().onTrue(new SequentialCommandGroup(
        new TimedIntakeSetPowerCommand(score, 10,0.75),
        new ScoringCommand(score, Constants.ScoringPos.STORE)));
    
    // source
    armController.y().onTrue(new SequentialCommandGroup(
        new ScoringCommand(score, Constants.ScoringPos.SOURCE),
        new IntakingCommand(score, 8),
        new ScoringCommand(score, Constants.ScoringPos.CLIMB),
        new InstantCommand(() -> score.ext.runAndResetEncoder())));
    
    // Preclimb
    armController.b().onTrue(new ScoringCommand(score, Constants.ScoringPos.CLIMB));

    /* ground, intake, hold */
    armController.a().onTrue(new SequentialCommandGroup(
        new ScoringCommand(score, Constants.ScoringPos.GROUND),
        new IntakingCommand(score, 12),
        new ScoringCommand(score, Constants.ScoringPos.STORE)));
    
    armController.x().onTrue(new ScoringCommand(score, Constants.ScoringPos.STORE));

    // Climber
    armController.pov(180).onTrue(new ClimbCommand(climbSub, 0));
    armController.pov(90).onTrue(new SequentialCommandGroup(
        new InstantCommand(() -> score.arm.setState(Constants.RobotState.CLIMBING)),
        new ScoringCommand(score, Constants.ScoringPos.STORE), 
        new WaitCommand(0.5),
        new ClimbCommand(climbSub, 90)
    ));
    armController.pov(0).onTrue(new ParallelCommandGroup(
        new ClimbCommand(climbSub, 90),
        new ScoringCommand(score, Constants.ScoringPos.CLIMB)
    ));
    armController.pov(270).onTrue(new SequentialCommandGroup(
        new ClimbCommand(climbSub, 25),
        new InstantCommand(() -> score.arm.setState(Constants.RobotState.DEFAULT))
    ));

    armController.leftBumper().onTrue(new ResetClimbCommand(climbSub));

    armController.rightBumper().onTrue(new InstantCommand(() -> score.ext.runAndResetEncoder()));

    armController.rightStick().onTrue(new InstantCommand( () -> score.setUseVelocityIntake(!score.getUseVelocityIntake()))); // I think this is on click

    /* GET DRIVE MOTOR ETC.
    SwerveDrivetrain drive = new SwerveDrivetrain(null, null);
    drive.getModule(0).getDriveMotor();
    */
    
  }

  public RobotContainer() {
    configureBindings();
  }

  public void logClimbStickyFaults() {
    climbSub.logMotorLeftStickyFaults();
    climbSub.logMotorRightStickyFaults();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

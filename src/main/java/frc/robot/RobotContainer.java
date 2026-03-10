// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.AlignAndDriveCommand;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {

  private final VisionSubsystem visionSubsystem = new VisionSubsystem();  
  private final ShooterSubsystem shootersubsystem = new ShooterSubsystem();
  private final HopperSubsystem hopperSubsystem = new HopperSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  private final SwerveSubsystem drivebase = new SwerveSubsystem();
  final CommandXboxController driverXbox = new CommandXboxController(DriveConstants.kDriverControllerPort);
  final CommandXboxController supportXbox = new CommandXboxController(DriveConstants.kOperatorControllerPort);

  private final SendableChooser<Command> autoChooser;

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                              () -> driverXbox.getLeftY()*-1,
                                                              () -> driverXbox.getLeftX()*-1)
                                                              .withControllerRotationAxis(() -> driverXbox.getRightX()*-1)
                                                              .deadband(DriveConstants.kDriveDeadband)
                                                              .scaleTranslation(0.7)
                                                              .allianceRelativeControl(false);
Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
() -> MathUtil.applyDeadband(driverXbox.getLeftY()*-1,DriveConstants.kDriveDeadband),
() -> MathUtil.applyDeadband(driverXbox.getLeftX()*-1,DriveConstants.kDriveDeadband),
() -> driverXbox.getRightX()*-1,
() -> driverXbox.getRightY()*-1);

Command driveFielOrientedAngularVelocity = drivebase.driveFieldOrientedCommand(driveAngularVelocity);
  public RobotContainer() {
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("AutoChooser", autoChooser);

    NamedCommands.registerCommand("AlignToTag", new RunCommand(() -> drivebase.alignToTag(visionSubsystem), drivebase).raceWith(new WaitCommand(5)));
    NamedCommands.registerCommand("RunShooter", shootersubsystem.runShootCommand());
    NamedCommands.registerCommand("StopShooter", shootersubsystem.stop());
    NamedCommands.registerCommand("RunHopper", hopperSubsystem.runHopper(.6, -.6));
    NamedCommands.registerCommand("StopHopper", hopperSubsystem.stop());
    NamedCommands.registerCommand("RunIntake", intakeSubsystem.runIntake(.9));
    NamedCommands.registerCommand("StopIntake", intakeSubsystem.stop());
    NamedCommands.registerCommand("ExtendIntake", intakeSubsystem.runExtension(.6));
    NamedCommands.registerCommand("StopExtendIntake", intakeSubsystem.stop());

  }

  private void configureBindings() {
    drivebase.setDefaultCommand(driveFielOrientedAngularVelocity);

    //driverXbox.a().whileTrue(drivebase.alignToTag(visionSubsystem));
    driverXbox.a().whileTrue(new AlignAndDriveCommand(visionSubsystem, drivebase, driverXbox));
    // driverXbox.y().onTrue(Commands.runOnce(drivebase::zeroGyro));
    driverXbox.b().onTrue(driveFielOrientedAngularVelocity);
    driverXbox.x().onTrue(driveFieldOrientedDirectAngle);

    driverXbox.y().onTrue(hopperSubsystem.runHopper(.6, -0.6)).onFalse(hopperSubsystem.stop());
    driverXbox.rightTrigger(0.1).onTrue(intakeSubsystem.runIntake(.9)).onFalse(intakeSubsystem.stop());
    driverXbox.rightBumper().whileTrue(shootersubsystem.runShootCommand()).onFalse(shootersubsystem.stop());
    //supportXbox.rightBumper().onTrue(shootersubsystem.setPower(0.3)).onFalse(shootersubsystem.stop());
    driverXbox.povUp().onTrue(new InstantCommand(() -> {
      shootersubsystem.increment_setpoint(25);
    }));
    driverXbox.povDown().onTrue(new InstantCommand(() -> {
      shootersubsystem.increment_setpoint(-25);
    }));
driverXbox.povRight().onTrue(new InstantCommand(() -> {
      double visionRPM = visionSubsystem.getrpm();
      if (visionRPM != 0)
      {
        shootersubsystem.set_setpoint(visionRPM);
      }
      
    }));

    driverXbox.back().onTrue(intakeSubsystem.runExtension(-.6)).onFalse(intakeSubsystem.stop());
    driverXbox.start().onTrue(intakeSubsystem.runExtension(.6)).onFalse(intakeSubsystem.stop());


  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

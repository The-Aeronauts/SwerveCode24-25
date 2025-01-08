// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.Map;
import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{


  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public static double Accel;
  
//Sets the max Accel of the bot with Slider in ShuffleBoard



  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    driveBinding();
        // Applies deadbands and inverts controls because joysticks
        // are back-right positive while robot
        // controls are front-left positive
        // left stick controls translation
        // right stick controls the rotational velocity 
        // buttons are quick rotation positions to different ways to face
        // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
        AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                       () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                                     OperatorConstants.LEFT_Y_DEADBAND),
                                                                       () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                                     OperatorConstants.LEFT_X_DEADBAND),
                                                                       () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                                     OperatorConstants.RIGHT_X_DEADBAND),
                                                                       driverXbox.getHID()::getYButtonPressed,
                                                                       driverXbox.getHID()::getAButtonPressed,
                                                                       driverXbox.getHID()::getXButtonPressed,
                                                                       driverXbox.getHID()::getBButtonPressed);
      }
      /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */

   /*ShuffleboardTab tab = Shuffleboard.getTab("Drive");
   GenericEntry maxAccel =
       tab.add("Max Accel FpsSq", Constants.MAX_ACCELTeleop)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", 0, "max", Constants.MAX_ACCELTeleop)) // specify widget properties here
      .getEntry()*/    

  public void driveBinding() {
   /* Accel = maxAccel.getDouble(Constants.MAX_ACCELTeleop);
    double Accelmps = Accel/6.25;
    SlewRateLimiter LYfilter = new SlewRateLimiter(Accelmps);
    SlewRateLimiter LXfilter = new SlewRateLimiter(Accelmps);*/
  
          // Applies deadbands and inverts controls because joysticks
          // are back-right positive while robot
          // controls are front-left positive
          // left stick controls translation
          // right stick controls the desired angle NOT angular rotation
        
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
              () -> MathUtil.applyDeadband(Controls.Direction * -1 * driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
              () -> MathUtil.applyDeadband(Controls.Direction * -1 * driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
              () -> driverXbox.getRightX() * -1 * Controls.Direction,
              () -> driverXbox.getRightY() * Controls.Direction);
  
          // Applies deadbands and inverts controls because joysticks
          // are back-right positive while robot
          // controls are front-left positive
          // left stick controls translation
          // right stick controls the angular velocity of the robot
  
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
              () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
              () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
              () -> driverXbox.getRightX() * 0.5);
  
    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
      () -> MathUtil.applyDeadband(Controls.Direction * -1 * driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(Controls.Direction * -1 * driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      //() -> driverXbox.getRawAxis(4) * 0.5);
      () -> driverXbox.getRightX() * -1 * Controls.Direction,
      () -> driverXbox.getRightY() * Controls.Direction);
  
  
  
  drivebase.setDefaultCommand(
      !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
  }

  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    //driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    driverXbox.x().whileTrue(
        Commands.deferredProxy(() -> drivebase.driveToPose(
                                   new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              ));
    driverXbox.b().whileTrue(
        Commands.deferredProxy(() -> drivebase.driveToPose(
                                   new Pose2d(new Translation2d(1.40, 5.55), Rotation2d.fromDegrees(0)))
                              ));
    driverXbox.y().whileTrue(drivebase.aimAtSpeaker(0.25));
    //driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("NewAuto");
  }
  
  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  

}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.XboxController;

import java.util.Map;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Controls
{
      
      public static double speedmax = 1; //12.5 MAX w/ L1 Ratio
      public static double swerveheading = 0;
      public static int swervespeedy;
      public static int swervespeedx;
      public static double drivemodeteleop;
      public static double drivemodeauto;
      public static double drivemodedisabled;
      public static double LeftX;
      public static double LeftY;
      public static double RightX;
      public static double RightY;
      public static double Forward;
      public static double Strafe;
      public static double Direction;

        /*static ShuffleboardTab tab = Shuffleboard.getTab("Drive");
                            static GenericEntry maxSpeed =
                                                    tab.add("Max Speed", 1)
                                 .withWidget(BuiltInWidgets.kNumberSlider)
                                 .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
                                 .getEntry();
                            
                                 public static double speedmax = maxSpeed.getDouble(1.0); //Units.feetToMeters(speedmaxfts);*/
}

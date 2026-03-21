// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kAuxControllerPort = 1;
    public static final double DEADBAND = .05;
  }

  public static class DriveLimits {
    public static final double MAX_SPEED = Units.feetToMeters(10); // theoretical: 14.63 Ft/s
    public static final double MEDIUM_SPEED_FACTOR = Units.feetToMeters(1);
  }

    public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
    public static final int kAuxControllerPort = 1;
    public static final double kAuxDeadband = 0.05;

    public static final double FASTTRANSLATIONSCALE = 1.00;
    public static final double FASTROTATIONSCALE = .5;

    public static final double SLOWTRANSLATIONSCALE = .2;
    public static final double SLOWROTATIONSCALE = .5;

    public static final double ORBITRADIUS = 1.90;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class ShooterConstants {
    public static final int SHOOTERMOTORID = 9;
    public static final double kP = .1;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double RPMATSIXVOLTS = 0; // figure out later
    public static final double kS = 0; //figure out later
    public static final double kV = ShooterConstants.RPMATSIXVOLTS / 6; //use sysid later

    // for state machine
    public static final double NORMALSHOOTINGVOLTAGE = 1.50;
    public static final double REVERSESHOOTINGVOLTAGE = 8.0;
    public static final double SLOWSHOOTINGVOLTAGE = 4.0;
    public static final double FASTSHOOTINGVOLTAGE = 3.5;


    public static final double NORMALSHOOTINGRADPERSEC = 4000;

    public static final double GEARRATIO = 1 / 1; // a/b, b rotations of the motor will give a rotation of the flywheel

    // constants for voltage guesstimation for any distance away from the hub
    

  }

  public static final class IntakeConstants {
    public static final int INTAKEMOTORID = 10;

    // for state machine
    public static final double INTAKINGFASTSPEED = 8.0;
    public static final double INTAKINGSLOWSPEED = 4.5;
    public static final double SPITOUTSPEED = 6.0;
    public static final double SHOOTSPEED = 8.0;
  }

  public static final class EndshotConstants {
    public static final int ENDSHOTMOTORID = 12;

    // for state machine
    public static final double SHOOTSPEED = 8.0;
    public static final double INTAKINGSPEED = 4.5;
    public static final double FASTSHOOTSPEED = 8.75;
  }

  public static final class ExtendablehopperConstants {
    public static final int EXTENDABLEHOPPERMOTORID = 13;

    // for state machine
    public static final double LETOUTVOLTS = 8.0;
    public static final double PULLINVOLTS = 4.5;
  }

  public static final class IndexerConstants {
    public static final int INDEXERMOTORID = 11;
    
    // for state machine
    public static final double REVERSEINDEXINGVOLTS = 5.0;
    public static final double INDEXINGVOLTS = 5.0;
    public static final double INDEXINGTARGETRPM = 1.0;
    public static final double kP = 5.0;
    public static final double kI = 0;
    public static final double kD = 0;

  }

  // public static final class ClimberConstants{
  //   public static final int CLIMBERMOTORLEFTID = 11;
  //   // public static final int CLIMBERMOTORRIGHTID = 12;
  //   public static final double CLIMBGEARRATIO = 125 / 1; // 20/1 rotations of the motor will be one rotation of the pulley
  //   public static final double PULLEYRADIUSMETERS = .022225; // meters -- PLACEHOLDER
  //   public static final double ROTATIONSTOMETERS = 9616.056;

  //   public static final boolean RIGHTMOTORINVERTEDFROMLEFT = true;

  //   // for state machine
  //   public static final double RAISECLAWSVOLTS = 9;
  //   public static final double LOWERCLAWSVOLTS = 9;

  //   // pid configuration
  //   public static final double MAXPIDVELOCITY = 7; // meters per sec
  //   public static final double MAXPIDACCELERATION = 7; // meters per sec^2
  //   public static final double kP = 75;
  //   public static final double kI = 0;
  //   public static final double kD = 0;
  //   // public static final double MAXCLIMBVOLTS = 8;

  //   // pid setpoints
  //   public static final double CLAWSTOTOPMETERS = .208;
  //   public static final double CLAWSTOROBOTLIFTEDMETERS = .17;
  //   public static final double CLAWSTOBOTTOMMETERS = 0;
  // }

  public static final class FieldPoses {
    public static final Pose2d BLUEHUBCENTER = new Pose2d(4.620, 4.025, new Rotation2d());
    public static final Pose2d REDHUBCENTER = new Pose2d(11.920, 4.025, new Rotation2d()); 
  }
}

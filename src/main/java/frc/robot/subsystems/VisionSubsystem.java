// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.LimelightHelpers.PoseEstimate;
import swervelib.SwerveDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.measure.AngularVelocity;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;

public class VisionSubsystem extends SubsystemBase {

  //  private static DigitalInput DIO_1;
  //  private boolean output1;
  //  private boolean[] sensorValues = new boolean[2];

  boolean initialLimelightPose = false;

  boolean hasTarget = false;

  boolean red;
  
    
  
  
    /** Creates a new VisionSubsystem. */
    public VisionSubsystem() {
  
  
      // DIO_1 = new DigitalInput(1);

    if (DriverStation.isFMSAttached()) {
      if (DriverStation.getAlliance().isPresent()) {
        red = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
      }
    }

    LimelightHelpers.SetIMUMode("limelight", 0);

  }


  public boolean getTV() {
    return LimelightHelpers.getTV("limelight");
  }

  public double getTX() {
    return LimelightHelpers.getTX("limelight");
  }

  public void updatePoseEstimator(SwerveDrive swerve) {
    PoseEstimate  poseEst = getEstimatedGlobalPose("limelight");
      if (poseEst != null) {
        swerve.addVisionMeasurement(poseEst.pose, poseEst.timestampSeconds);
      }
  }

  /**THIS IS THE METHOD THAT IS BEING CALLED BY PERIODIC IN OUR SWERVE DRIVE */
  public void updatePosesEstimator(SwerveDrive swerve) {
    double maxta = 0.4;
    String camera = null;

    String[] limelights = {"limelight"}; // , "limelight-rear"
    // String[] limelights = {"limelight-right"};
    for (String limelight: limelights) {
      if (LimelightHelpers.getTV(limelight) && LimelightHelpers.getTA(limelight) > maxta) {
        maxta = LimelightHelpers.getTA(limelight);
        camera = limelight;
      }
    }
    if (camera != null) {
      PoseEstimate poseEst = getEstimatedGlobalPose(camera);
      swerve.addVisionMeasurement(poseEst.pose, poseEst.timestampSeconds);
      SmartDashboard.putBoolean("limelightTV", true);
      hasTarget = true;
    } else {
      hasTarget = false;
      SmartDashboard.putBoolean("limelightTV", false);
    }
  }


  /**
   * Building this out for hopefully a quick test. to try mega tag 2
   * @param swerve
   */
  public void updatePosesEstimatorMT2(SwerveDrive swerve) {

    double maxta = 0;
    String camera = null;
    PoseEstimate mt2 = new PoseEstimate();
    String[] limelights = {"limelight"}; // , "limelight-rear"
    for (String limelight: limelights) {
      LimelightHelpers.PoseEstimate megaTag2Pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight);
      LimelightHelpers.getBotPose2d_wpiBlue(limelight);
      
      if (megaTag2Pose != null) {
        if(megaTag2Pose.tagCount > 0)
        {
          //we have a tag!
          //if the TA is larger than the other camera
          if(LimelightHelpers.getTA(limelight) > maxta)
          {
            maxta = LimelightHelpers.getTA(limelight);
            mt2  = megaTag2Pose;
            camera = limelight;
          }

        }
      }

    }
    if (camera != null) {
      swerve.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
      SmartDashboard.putBoolean("limelightTV", true);
    }
    else {
      SmartDashboard.putBoolean("limelightTV", false);
    }
  }
    
  public PoseEstimate getEstimatedGlobalPose(String limelight) {
    if (LimelightHelpers.getTV(limelight)) {
      hasTarget = true;
      PoseEstimate poseEst = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight);
     
      SmartDashboard.putBoolean("limelightTV", LimelightHelpers.getTV(limelight));
      SmartDashboard.putNumber("limelightX", poseEst.pose.getX());
      SmartDashboard.putNumber("limelightY", poseEst.pose.getY());
      return poseEst;
    }
    SmartDashboard.putBoolean("limelightTV", LimelightHelpers.getTV(limelight));
    SmartDashboard.putNumber("limelightX", new PoseEstimate().pose.getX());
    SmartDashboard.putNumber("limelightY", new PoseEstimate().pose.getY());
    return new PoseEstimate(); // IDK abt ths
  }

  public PoseEstimate[] getEstimatedGlobalPose(String[] limelights) {
    PoseEstimate[] poseEsts = new PoseEstimate[limelights.length];
    int num = 0;
    for (String limelight : limelights) {
      if (LimelightHelpers.getTV(limelight)) {
        PoseEstimate poseEst = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight);
        poseEsts[num] = poseEst;
      }
      else {
        poseEsts[num] = null;
      }
      num++;
    }
    return poseEsts;
     // IDK abt ths
  }
  public void updateLimelightYaw(SwerveSubsystem swerve) {

    double[] stddevs = NetworkTableInstance.getDefault().getTable("limelight")
                          .getEntry("stddevs").getDoubleArray(new double[12]);
    double LL4yaw = LimelightHelpers.getIMUData("limelight").Yaw;
    NetworkTableInstance.getDefault().getTable("Limelight stuff").getEntry("Stddevs").setDoubleArray(stddevs);
    NetworkTableInstance.getDefault().getTable("Limelight stuff").getEntry("Stddevs[5]").setDouble(stddevs[5]);
    NetworkTableInstance.getDefault().getTable("Limelight stuff").getEntry("LL4 Yaw").setDouble(LL4yaw);

    if (stddevs[5] < 1.5) {
      LimelightHelpers.SetRobotOrientation("limelight", 
          LimelightHelpers.getBotPose2d_wpiBlue("limelight").getRotation().getDegrees(), 0, 0, 0, 0, 0);
    } else {
      LimelightHelpers.SetRobotOrientation("limelight", 
          swerve.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      SmartDashboard.putBoolean("Limelight Yaw Type", false);
    }
  }
    
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    
    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    
    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("Limelight'X'", getTX());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


}
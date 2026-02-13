// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.IntakeStateSubsystem;
import frc.robot.subsystems.ShooterStateSubsystem;
import frc.robot.subsystems.IntakeStateSubsystem.IntakeState;
import frc.robot.subsystems.ShooterStateSubsystem.ShooterState;
import swervelib.SwerveInputStream;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // Replace with CommandPS4Controller or CommandJoystick if needed
  // IntakeStateSubsystem intakeSub = new IntakeStateSubsystem();
  // ShooterStateSubsystem shooterSub = new ShooterStateSubsystem();

  // The robot's subsystems and commands are defined here...
  private VisionSubsystem vision = new VisionSubsystem();
  private final SwerveSubsystem driveBase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "maxSwerve/DOTM2025"), vision); // where to configure the robot or "choose" it
  private final SendableChooser<Command> autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // private final CommandXboxController m_auxController =
  //     new CommandXboxController(OperatorConstants.kAuxControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    DriverStation.silenceJoystickConnectionWarning(true);

    // Configure the trigger bindings
    configureBindings();
    configureNamedCommands();
    
    autoChooser = AutoBuilder.buildAutoChooser(); //pick a default

    autoChooser.addOption("New Auto", new PathPlannerAuto("New Auto"));

    SmartDashboard.putData("Auto Chooser", autoChooser);
    //added because purdue did it
    SmartDashboard.putData(CommandScheduler.getInstance());

    //assign different values to the suppliers in the Supplier Registry
    assignSuppliers();

    //put suppliers in the subsystems they need to be in
    transportSuppliers();
  }

  SwerveInputStream driveAngularVelocity  = SwerveInputStream.of(driveBase.getSwerveDrive(),
                                      () -> -m_driverController.getLeftY(), 
                                      () -> -m_driverController.getLeftX())
                                      .withControllerRotationAxis(() -> -m_driverController.getRightX())
                                      .deadband(OperatorConstants.DEADBAND)
                                      .scaleTranslation(0.8)
                                      .allianceRelativeControl(true);
             
  // same as normal drive angular velocity but uses the PID Rotation function to align with the hub
  SwerveInputStream driveAngularVelocityWithAngleHubAlignment  = SwerveInputStream.of(driveBase.getSwerveDrive(),
                                          () -> -m_driverController.getLeftY(), 
                                          () -> -m_driverController.getLeftX())
                                          .withControllerRotationAxis(() -> driveBase.getHubAlignmentRotationalPIDOutput())
                                          .deadband(OperatorConstants.DEADBAND)
                                          .scaleTranslation(0.8)
                                          .allianceRelativeControl(true);

  // orbits the robot around the hub
  SwerveInputStream driveAngularVelocityOrbitHub  = SwerveInputStream.of(driveBase.getSwerveDrive(),
                                          () -> driveBase.getXAxisToOrbitPIDOutput(), 
                                          () -> driveBase.getYAxisToOrbitPIDOutput() + (Math.cos(driveBase.getAngleToHub()) * -m_driverController.getLeftX())) //will add joystick control scaled to the orbit speed we want
                                          .withControllerRotationAxis(() -> driveBase.getHubAlignmentRotationalPIDOutput())
                                          .deadband(OperatorConstants.DEADBAND)
                                          .scaleTranslation(1.0)
                                          .allianceRelativeControl(true);

  SwerveInputStream driveRegular = driveAngularVelocity.copy().scaleTranslation(1.05);

  SwerveInputStream driveAngularVelocitySlow = driveAngularVelocity.copy().scaleTranslation(Constants.DriveLimits.MEDIUM_SPEED_FACTOR);


  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                                                          .withControllerHeadingAxis(m_driverController::getRightX, 
                                                          m_driverController::getRightY).headingWhile(false);
                                                        //withControllerHeadingAxis(m_driverController::getRightX, m_driverController::getRightX  <- change this to Y for special mode

  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                          .allianceRelativeControl(false);




  Command driveFieldOrientedDirectAngle = driveBase.driveFieldOriented(driveDirectAngle);
  // Command driveFieldOrientedAngularVelocity = driveBase.driveFieldOriented(driveRegular); // Normal Drive
  Command driveFieldOrientedAngularVelocity = driveBase.driveFieldOriented(driveRegular); // Normal Drive
  Command driveFieldOrientedAngularVelocitySlow = driveBase.driveFieldOriented(driveAngularVelocitySlow); 

  Command driveFieldOrientedAngularVelocityWithAngleHubAlignment = driveBase.driveFieldOriented(driveAngularVelocityWithAngleHubAlignment);
  Command driveFieldOrientedAngularVelocityOrbitHub = driveBase.driveFieldOriented(driveAngularVelocityOrbitHub);



  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // m_driverController.a()
    //   .onTrue(shooterSub.setStateCmd(ShooterState.NORMALSHOOTING))
    //   .onFalse(shooterSub.setStateCmd(ShooterState.IDLE));


    // m_driverController.b()
    //   .onTrue(intakeSub.setStateCmd(IntakeState.INTAKING))
    //   .onFalse(intakeSub.setStateCmd(IntakeState.IDLE));

    m_driverController.y()
      .toggleOnTrue(driveFieldOrientedAngularVelocityWithAngleHubAlignment)
      .toggleOnFalse(driveFieldOrientedAngularVelocity);

    m_driverController.x().onTrue(driveBase.zeroHeadingCommand());

    // m_driverController.x()
    //   .toggleOnTrue(driveFieldOrientedAngularVelocityOrbitHub)
    //   .toggleOnFalse(driveFieldOrientedAngularVelocity);

    // m_auxController.a()
    //   .onTrue(shooterSub.setStateCmd(ShooterState.JOYSTICKCONTROL));

    // set to default drive, simulation code commented out
    // if (RobotBase.isSimulation())
    // {
    //   driveBase.setDefaultCommand(driveFieldOrientedAngularVelocityKeyboard);
    // } 
    // else 
    // {
      driveBase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    // }
    


  }

  private void configureNamedCommands() {

  }

  private void assignSuppliers(){
    SupplierRegistry.distanceToHub = 
      () -> driveBase.getDistanceToHub();
    // SupplierRegistry.auxLeftY = 
    //   () -> m_auxController.getLeftY();
  }

  private void transportSuppliers(){
    // shooterSub.setDistanceToHubSupplier(SupplierRegistry.distanceToHub);
    // shooterSub.setAuxLeftY(SupplierRegistry.auxLeftY);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();  

  }
}

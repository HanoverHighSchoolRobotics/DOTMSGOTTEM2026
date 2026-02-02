// package frc.robot.subsystems;

// import static edu.wpi.first.units.Units.Volts;
// import static edu.wpi.first.units.Units.Radians;
// import static edu.wpi.first.units.Units.RadiansPerSecond;

// import com.revrobotics.PersistMode;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.ResetMode;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkLowLevel.MotorType;

// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.units.measure.AngularVelocity;
// import edu.wpi.first.units.measure.Voltage;
// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
// import frc.robot.Configs;
// import frc.robot.Constants.ShooterConstants;

// public class ShooterSubsystem extends SubsystemBase{
    
//     private SparkMax motor;
//     private RelativeEncoder encoder;

//   private final SysIdRoutine sysIdRoutine =
//       new SysIdRoutine(
//           new SysIdRoutine.Config(
//           ),
//           new SysIdRoutine.Mechanism(
//               this::setVoltage,
//               this::logData,
//               this
//           )
//       );

//     public ShooterSubsystem() {
//         this.motor = new SparkMax(ShooterConstants.SHOOTERMOTORID, MotorType.kBrushless);

//         motor.configure(Configs.IntakeConfigs.intakeMotorConfig, ResetMode.kResetSafeParameters,
//             PersistMode.kPersistParameters);

//         this.encoder = motor.getEncoder();
//     }

//     //methods for sysid to use
//     private void setVoltage(Voltage volts){
//       motor.setVoltage(volts);
//     }

//     private void logData(SysIdRoutineLog log){
//       AngularVelocity velocityRadPerSec = RadiansPerSecond.of(Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity()));
//       Angle position = Radians.of(Units.rotationsToRadians(encoder.getPosition()));
//       Voltage appliedVoltage = Volts.of(motor.get() * RobotController.getBatteryVoltage());

//       log.motor("flywheel")
//           .voltage(appliedVoltage)
//           .angularPosition(position)
//           .angularVelocity(velocityRadPerSec);
//     }

//   // Commands to run tests
//   public Command sysIdQuasistaticForward() {
//     return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
//   }

//   public Command sysIdQuasistaticReverse() {
//     return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
//   }

//   public Command sysIdDynamicForward() {
//     return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
//   }

//   public Command sysIdDynamicReverse() {
//     return sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
//   }
// }

// package frc.robot.subsystems;

// import com.revrobotics.PersistMode;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.ResetMode;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkLowLevel.MotorType;

// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Configs;
// import frc.robot.Constants.ShooterConstants;

// public class ShooterSubsystem extends SubsystemBase{
//     private SparkMax motor;
//     private RelativeEncoder encoder;

//     // ProfiledPIDController shooterPID = new ProfiledPIDController(
//     //     ShooterConstants.SHOOTERkP, 0, 0,
//     //     new TrapezoidProfile.Constraints(ShooterConstants.MAXPIDVELOCITY, ShooterConstants.MAXPIDACCELERATION));

//     // private double desiredSetpoint = 0;

//     public ShooterSubsystem(){
//         this.motor = new SparkMax(ShooterConstants.SHOOTERMOTORID, MotorType.kBrushless);

//         motor.configure(Configs.ShooterConfigs.shooterMotorConfig, ResetMode.kResetSafeParameters,
//             PersistMode.kPersistParameters);

//         this.encoder = motor.getEncoder();
//     }

//     public void setShooterSpeed(double speed){
//         motor.set(speed);
//     }
    
//     public double getEncoderPos(){
//         return encoder.getPosition();
//     }

//     public double getEncoderVel(){
//         return encoder.getVelocity();
//     }

//     public void periodic(){
//         // SmartDashboard.putNumber("Shooter Encoder Position", getEncoderPos());

//         // setShooterSpeed(shooterPID.calculate(getEncoderPos(), desiredSetpoint));
//     }


//     public Command setShooterCmd(double speed){
//         return runOnce(
//         () -> setShooterSpeed(speed)
//         );
//     }
// }
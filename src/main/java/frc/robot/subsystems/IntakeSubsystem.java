// package frc.robot.subsystems;

// import com.revrobotics.PersistMode;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Configs;
// import frc.robot.Constants.IntakeConstants;

// public class IntakeSubsystem extends SubsystemBase {
//     private SparkMax motor;
//     private RelativeEncoder encoder;

//     public IntakeSubsystem() {
//         this.motor = new SparkMax(IntakeConstants.INTAKEMOTORID, MotorType.kBrushless);

//         motor.configure(Configs.IntakeConfigs.intakeMotorConfig, ResetMode.kResetSafeParameters,
//             PersistMode.kPersistParameters);

//         this.encoder = motor.getEncoder();
//     }

//     public void setIntakeSpeed(double speed){
//         motor.set(speed);
//     }

//     public double getEncoderPos(){
//         return encoder.getPosition();
//     }

//     @Override
//     public void periodic() {
//     //    SmartDashboard.putNumber("Intake Encoder Position", getEncoderPos()); 
//     }

//     public Command setIntakeCmd(double speed){
//         return runOnce(
//         () -> setIntakeSpeed(speed)
//         );
//     }

// }

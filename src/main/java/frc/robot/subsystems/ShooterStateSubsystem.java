package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ShooterConstants;

public class ShooterStateSubsystem extends SubsystemBase {
    private SparkMax motor;
    private RelativeEncoder encoder;

    public enum ShooterState {
        IDLE,
        NORMALSHOOTING,
        DISTANCEDEPENDENT,
        REVERSE
    }

    private ShooterState state;

    public ShooterStateSubsystem() {
        // setup
        this.motor = new SparkMax(ShooterConstants.SHOOTERMOTORID, MotorType.kBrushless);

        motor.configure(Configs.ShooterConfigs.shooterMotorConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        this.encoder = motor.getEncoder();

        // start on idle
        state = ShooterState.IDLE;
    }

    // basic functionality
    public void setShooterSpeed(double speed){
        motor.set(speed);
    }

    public double getEncoderPos(){
        return encoder.getPosition();
    }

    @Override
    public void periodic() {

        switch(state){
            case IDLE:
                idlePeriodic();
                break;
            case NORMALSHOOTING:
                normalshootingPeriodic();
                break;
            case DISTANCEDEPENDENT:
                distancedependentPeriodic();
                break;
            case REVERSE:
                reversePeriodic();
                break;
        }
    }

    //State periodic functions declared here
    public void idlePeriodic(){
        setShooterSpeed(0);
    }

    public void normalshootingPeriodic(){
        setShooterSpeed(ShooterConstants.NORMALSHOOTINGSPEED);
    }

    public void distancedependentPeriodic(){

    }

    public void reversePeriodic(){
        setShooterSpeed(-1 * ShooterConstants.REVERSESHOOTINGSPEED);
    }

    // change state method and command
    public void setState(ShooterState newState){
        this.state = newState;
    }

    public Command setStateCmd(ShooterState newState){
        SmartDashboard.putString("ShooterState", state.toString());
        return runOnce(
            () -> setState(newState)
        );
    }
}

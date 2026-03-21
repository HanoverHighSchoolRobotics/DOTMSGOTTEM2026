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
import frc.robot.Constants.EndshotConstants;

public class EndshotStateSubsystem extends SubsystemBase {
    private SparkMax motor;
    private RelativeEncoder encoder;

    public enum EndshotState {
        IDLE,
        SHOOT,
        INTAKING,
        FASTSHOOT
    }

    private EndshotState state;

    public EndshotStateSubsystem() {
        // setup
        this.motor = new SparkMax(EndshotConstants.ENDSHOTMOTORID, MotorType.kBrushless);

        motor.configure(Configs.EndshotConfigs.endshotMotorConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        this.encoder = motor.getEncoder();

        // start on idle
        state = EndshotState.IDLE;

        SmartDashboard.putString("EndshotState", this.state.toString());
    }

    // basic functionality
    public void setEndshotVoltage(double speed){
        motor.setVoltage(speed);
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
            case SHOOT:
                shootPeriodic();
                break;
            case INTAKING:
                intakingPeriodic();
                break;
            case FASTSHOOT:
                fastshootPeriodic();
                break;
        }
    }

    //State periodic functions declared here
    public void idlePeriodic(){
        setEndshotVoltage(0);
    }

    public void shootPeriodic(){
        setEndshotVoltage(EndshotConstants.SHOOTSPEED);
    }

    public void intakingPeriodic(){
        setEndshotVoltage(-1 * EndshotConstants.INTAKINGSPEED);
    }

    public void fastshootPeriodic(){
        setEndshotVoltage(EndshotConstants.FASTSHOOTSPEED);
    }
    // change state method and command
    public void setState(EndshotState newState){
        this.state = newState;
        SmartDashboard.putString("EndshotState", this.state.toString());
    }

    public Command setStateCmd(EndshotState newState){
        return runOnce(
            () -> setState(newState)
        );
    }
}

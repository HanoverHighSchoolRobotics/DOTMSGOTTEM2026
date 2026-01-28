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
import frc.robot.Constants.IntakeConstants;

public class IntakeStateSubsystem extends SubsystemBase {
    private SparkMax motor;
    private RelativeEncoder encoder;

    public enum IntakeState {
        IDLE,
        INTAKING,
        SPITOUT
    }

    private IntakeState state;

    public IntakeStateSubsystem() {
        // setup
        this.motor = new SparkMax(IntakeConstants.INTAKEMOTORID, MotorType.kBrushless);

        motor.configure(Configs.IntakeConfigs.intakeMotorConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        this.encoder = motor.getEncoder();

        // start on idle
        state = IntakeState.IDLE;
    }

    // basic functionality
    public void setIntakeSpeed(double speed){
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
            case INTAKING:
                intakingPeriodic();
                break;
            case SPITOUT:
                spitoutPeriodic();
                break;
        }
    }

    //State periodic functions declared here
    public void idlePeriodic(){
        setIntakeSpeed(0);
    }

    public void intakingPeriodic(){
        setIntakeSpeed(IntakeConstants.INTAKINGSPEED);
    }

    public void spitoutPeriodic(){
        setIntakeSpeed(-1 * IntakeConstants.SPITOUTSPEED);
    }

    // change state method and command
    public void setState(IntakeState newState){
        this.state = newState;
    }

    public Command setStateCmd(IntakeState newState){
        SmartDashboard.putString("IntakeState", state.toString());
        return runOnce(
            () -> setState(newState)
        );
    }
}

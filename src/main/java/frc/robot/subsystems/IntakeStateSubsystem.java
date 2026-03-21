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
        INTAKINGFAST,
        INTAKINGSLOW,
        SPITOUT,
        SHOOT
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

        SmartDashboard.putString("IntakeState", this.state.toString());
    }

    // basic functionality
    public void setIntakeVoltage(double speed){
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
            case INTAKINGFAST:
                intakingfastPeriodic();
                break;
            case INTAKINGSLOW:
                intakingslowPeriodic();
                break;
            case SPITOUT:
                spitoutPeriodic();
                break;
            case SHOOT:
                shootPeriodic();
                break;
        }
    }

    //State periodic functions declared here
    public void idlePeriodic(){
        setIntakeVoltage(0);
    }

    public void intakingfastPeriodic(){
        setIntakeVoltage(IntakeConstants.INTAKINGFASTSPEED);
    }

    public void intakingslowPeriodic(){
        setIntakeVoltage(IntakeConstants.INTAKINGSLOWSPEED);
    }

    public void spitoutPeriodic(){
        setIntakeVoltage(-1 * IntakeConstants.SPITOUTSPEED);
    }

    public void shootPeriodic(){
        setIntakeVoltage(IntakeConstants.SHOOTSPEED);
    }

    // change state method and command
    public void setState(IntakeState newState){
        this.state = newState;
        SmartDashboard.putString("IntakeState", this.state.toString());
    }

    public Command setStateCmd(IntakeState newState){
        return runOnce(
            () -> setState(newState)
        );
    }
}

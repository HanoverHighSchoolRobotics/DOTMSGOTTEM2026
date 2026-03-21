package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
    private SparkMax motor;
    private RelativeEncoder encoder;

    private PIDController pid = new PIDController(IndexerConstants.kP, IndexerConstants.kI, IndexerConstants.kD);

    public enum IndexerState {
        IDLE,
        INDEXING,
        PIDINDEXING,
        REVERSEINDEXING
    }

    private IndexerState state;

    public IndexerSubsystem() {
        // setup
        this.motor = new SparkMax(IndexerConstants.INDEXERMOTORID, MotorType.kBrushless);

        motor.configure(Configs.IndexerConfigs.indexerMotorConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        this.encoder = motor.getEncoder();

        // start on idle
        state = IndexerState.IDLE;

        SmartDashboard.putString("IndexerState", this.state.toString());
    }

    // basic functionality
    public void setIndexerVoltage(double speed){
        motor.setVoltage(speed);
    }

    public double getEncoderPos(){
        return encoder.getPosition();
    }

    public double getEncoderVelocity(){
        return encoder.getVelocity();
    }

    @Override
    public void periodic() {

        switch(state){
            case IDLE:
                idlePeriodic();
                break;
            case INDEXING:
                indexingPeriodic();
                break;
            case PIDINDEXING:
                pidindexingPeriodic();
                break;
            case REVERSEINDEXING:
                reverseindexingPeriodic();
                break;
        }
    }

    //State periodic functions declared here
    public void idlePeriodic(){
        setIndexerVoltage(0);
    }

    public void indexingPeriodic(){
        setIndexerVoltage(IndexerConstants.INDEXINGVOLTS);
    }

    public void pidindexingPeriodic(){
        setIndexerVoltage(IndexerConstants.INDEXINGVOLTS + pid.calculate(getEncoderVelocity(), IndexerConstants.INDEXINGTARGETRPM));
    }

    public void reverseindexingPeriodic(){
        setIndexerVoltage(-1 * IndexerConstants.REVERSEINDEXINGVOLTS);
    }

    // change state method and command
    public void setState(IndexerState newState){
        this.state = newState;
        SmartDashboard.putString("IndexerState", this.state.toString());
    }

    public Command setStateCmd(IndexerState newState){
        return runOnce(
            () -> setState(newState)
        );
    }
}

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ExtendablehopperConstants;
import frc.robot.Configs;

public class ExtendablehopperSubsystem extends SubsystemBase {
    private SparkMax motor;
    private RelativeEncoder encoder;

    public enum ExtendablehopperState {
        IDLE,
        LETOUT,
        PULLIN
    }

    private ExtendablehopperState state;

    public ExtendablehopperSubsystem() {
        // setup
        this.motor = new SparkMax(ExtendablehopperConstants.EXTENDABLEHOPPERMOTORID, MotorType.kBrushless);

        motor.configure(Configs.ExtendablehopperConfigs.extendablehopperConfigs, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        this.encoder = motor.getEncoder();

        // start on idle
        state = ExtendablehopperState.IDLE;

        SmartDashboard.putString("ExtendablehopperState", this.state.toString());
    }

    // basic functionality
    public void setExtendablehopperVoltage(double speed){
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
            case LETOUT:
                letoutPeriodic();
                break;
            case PULLIN:
                pullinPeriodic();
                break;
        }
    }

    //State periodic functions declared here
    public void idlePeriodic(){
        setExtendablehopperVoltage(0);
    }

    public void letoutPeriodic(){
        setExtendablehopperVoltage(ExtendablehopperConstants.LETOUTVOLTS);
    }

    public void pullinPeriodic(){
        setExtendablehopperVoltage(-1 * ExtendablehopperConstants.PULLINVOLTS);
    }

    // change state method and command
    public void setState(ExtendablehopperState newState){
        this.state = newState;
        // handles any actions that need to be done upon a state being entered, like resetting pid
        onStateEnter(this.state);
        // post what state we are in
        SmartDashboard.putString("ExtendablehopperState", this.state.toString());
    }

    public Command setStateCmd(ExtendablehopperState newState){
        return runOnce(
            () -> setState(newState)
        );
    }

    // runs when a state is being entered
    public void onStateEnter(ExtendablehopperState state){
        switch(state){
            case IDLE:
                break;
            case LETOUT:
                break;
            case PULLIN:
                break;
        }
    }
}

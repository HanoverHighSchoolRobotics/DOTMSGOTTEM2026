package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ClimberConstants;

public class ClimberStateSubsystem extends SubsystemBase {
    private SparkMax leftMasterMotor;
    private SparkMax rightSlaveMotor;
    private RelativeEncoder encoder;

    // our pid will be used in meters (for elevator height) and seconds
    private ProfiledPIDController pid;

    public enum ClimberState {
        IDLE,
        RAISECLAWS,
        LOWERCLAWS,
        PIDCLAWSTOTOP,
        PIDCLAWSTOROBOTLIFTED,
        PIDCLAWSTOBOTTOM
    }

    private ClimberState state;

    public ClimberStateSubsystem() {
        // setup
        this.leftMasterMotor = new SparkMax(ClimberConstants.CLIMBERMOTORLEFTID, MotorType.kBrushless);
        this.rightSlaveMotor = new SparkMax(ClimberConstants.CLIMBERMOTORRIGHTID, MotorType.kBrushless);

        this.leftMasterMotor.configure(Configs.ClimberConfigs.climberLeftMotorConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        this.rightSlaveMotor.configure(Configs.ClimberConfigs.climberRightMotorConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        this.encoder = leftMasterMotor.getEncoder();

        this.pid = new ProfiledPIDController(ClimberConstants.kP, ClimberConstants.kI, ClimberConstants.kD, 
            new TrapezoidProfile.Constraints(ClimberConstants.MAXPIDVELOCITY, ClimberConstants.MAXPIDACCELERATION));

        this.pid.reset(getClimberHeightMeters());
        // start on idle
        this.state = ClimberState.IDLE;
    }

    // basic functionality
    public void setClimberVoltage(double volts){
        leftMasterMotor.setVoltage(volts);
    }

    public double getEncoderPositionRadians(){
        return Units.rotationsToRadians(encoder.getPosition());
    }

    // returns the current calculated height of the elevator in meters
    public double getClimberHeightMeters(){
        return getEncoderPositionRadians() * ClimberConstants.PULLEYRADIUSMETERS * ClimberConstants.CLIMBGEARRATIO;
    }

    @Override
    public void periodic() {

        switch(state){
            case IDLE:
                idlePeriodic();
                break;
            case RAISECLAWS:
                raiseclawsPeriodic();
                break;
            case LOWERCLAWS:
                lowerclawsPeriodic();
                break;
            case PIDCLAWSTOTOP:
                pidclawstotopPeriodic();
                break;
            case PIDCLAWSTOROBOTLIFTED:
                pidclawstorobotliftedPeriodic();
                break;
            case PIDCLAWSTOBOTTOM:
                pidclawstobottomPeriodic();
                break;
        }
    }

    //State periodic functions declared here
    public void idlePeriodic(){
        setClimberVoltage(0);
    }

    public void raiseclawsPeriodic(){
        setClimberVoltage(ClimberConstants.RAISECLAWSVOLTS);
    }

    public void lowerclawsPeriodic(){
        setClimberVoltage(-1 * ClimberConstants.LOWERCLAWSVOLTS);
    }

    public void pidclawstotopPeriodic(){
        setClimberVoltage(pid.calculate(getClimberHeightMeters(), ClimberConstants.CLAWSTOTOPMETERS));
    }

    public void pidclawstorobotliftedPeriodic(){
        setClimberVoltage(pid.calculate(getClimberHeightMeters(), ClimberConstants.CLAWSTOROBOTLIFTEDMETERS));
    }

    public void pidclawstobottomPeriodic(){
        setClimberVoltage(pid.calculate(getClimberHeightMeters(), ClimberConstants.CLAWSTOBOTTOMMETERS));
    }

    // change state method and command
    public void setState(ClimberState newState){
        this.state = newState;
        // handles any actions that need to be done upon a state being entered, like resetting pid
        onStateEnter(this.state);
        // post what state we are in
        SmartDashboard.putString("ClimberState", this.state.toString());
    }

    public Command setStateCmd(ClimberState newState){
        return runOnce(
            () -> setState(newState)
        );
    }

    // runs when a state is being entered
    public void onStateEnter(ClimberState state){
        switch(state){
            case IDLE:
                break;
            case RAISECLAWS:
                break;
            case LOWERCLAWS:
                break;
            case PIDCLAWSTOTOP:
                pid.reset(getClimberHeightMeters());
                break;
            case PIDCLAWSTOROBOTLIFTED:
                pid.reset(getClimberHeightMeters());
                break;
            case PIDCLAWSTOBOTTOM:
                pid.reset(getClimberHeightMeters());
                break;
        }
    }
}

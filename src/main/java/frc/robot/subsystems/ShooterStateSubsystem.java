package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ShooterConstants;

public class ShooterStateSubsystem extends SubsystemBase {
    private SparkMax motor;
    private RelativeEncoder encoder;

    // this pid and feed forward will be used in radians (for the flywheel) and seconds
    private PIDController pid = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV);

    //suppliers
    private DoubleSupplier distanceToHubSupplier;
    private DoubleSupplier auxLeftY;


    public enum ShooterState {
        IDLE,
        NORMALSHOOTING,
        JOYSTICKCONTROL,
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
    public void setShooterVoltage(double voltage){
        motor.setVoltage(voltage);
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
            case JOYSTICKCONTROL:
                joystickcontrolPeriodic();
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
        setShooterVoltage(0);
    }

    public void normalshootingPeriodic(){
        setShooterVoltage(ShooterConstants.NORMALSHOOTINGVOLTAGE);

        // pid version, please use once you get sysid working
        // setShooterVoltage(
        //     pid.calculate(getCurrentRadPerSec(), ShooterConstants.NORMALSHOOTINGRADPERSEC) 
        //     + feedforward.calculate(ShooterConstants.NORMALSHOOTINGRADPERSEC));
    }

    public void joystickcontrolPeriodic(){
        setShooterVoltage(auxLeftY.getAsDouble() * 10.0); //takes joystick value (-1 to 1) and multiplies by 10 volts
    }

    public void distancedependentPeriodic(){
        
    }

    public void reversePeriodic(){
        setShooterVoltage(-1 * ShooterConstants.REVERSESHOOTINGVOLTAGE);
    }

    // change state method and command
    public void setState(ShooterState newState){
        this.state = newState;
        SmartDashboard.putString("ShooterState", this.state.toString());
    }

    public Command setStateCmd(ShooterState newState){
        return runOnce(
            () -> setState(newState)
        );
    }

    //supplier gathering commands
    public void setDistanceToHubSupplier(DoubleSupplier dist){
        this.distanceToHubSupplier = dist;
    }

    public void setAuxLeftY(DoubleSupplier val){
        this.auxLeftY = val;
    }

    //data gathering commands
    public double getCurrentRadPerSec(){
        return Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());
    }
}

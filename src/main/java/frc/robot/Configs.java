package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ClimberConstants;



public final class Configs {
    public static final class ShooterConfigs {
        public static final SparkMaxConfig shooterMotorConfig = new SparkMaxConfig();

        static {
            shooterMotorConfig
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(50);
        }
    }

    public static final class IntakeConfigs {
        public static final SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();

        static {
            intakeMotorConfig
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(50);
        }
    }

    public static final class ClimberConfigs {
        public static final SparkMaxConfig climberLeftMotorConfig = new SparkMaxConfig();
        public static final SparkMaxConfig climberRightMotorConfig = new SparkMaxConfig();

        static {
            climberLeftMotorConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50);
            climberRightMotorConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50)
                .follow(ClimberConstants.CLIMBERMOTORLEFTID, ClimberConstants.RIGHTMOTORINVERTEDFROMLEFT);
        }
    }
}
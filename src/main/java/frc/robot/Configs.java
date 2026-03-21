package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;



public final class Configs {
    public static final class ShooterConfigs {
        public static final SparkMaxConfig shooterMotorConfig = new SparkMaxConfig();

        static {
            shooterMotorConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50);
        }
    }

    public static final class IntakeConfigs {
        public static final SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();

        static {
            intakeMotorConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50)
                .inverted(true);
        }
    }

    public static final class EndshotConfigs {
        public static final SparkMaxConfig endshotMotorConfig = new SparkMaxConfig();

        static {
            endshotMotorConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(60)
                .inverted(true);
        }
    }

    public static final class ExtendablehopperConfigs {
        public static final SparkMaxConfig extendablehopperConfigs = new SparkMaxConfig();

        static {
            extendablehopperConfigs
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50)
                .inverted(true);
        }
    }

    public static final class IndexerConfigs {
        public static final SparkMaxConfig indexerMotorConfig = new SparkMaxConfig();

        static {
            indexerMotorConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(60)
                .inverted(false);
        }
    }

    public static final class ClimberConfigs {
        public static final SparkMaxConfig climberLeftMotorConfig = new SparkMaxConfig();
        // public static final SparkMaxConfig climberRightMotorConfig = new SparkMaxConfig();

        static {
            climberLeftMotorConfig
                .idleMode(IdleMode.kBrake)
                .inverted(true);
            // climberRightMotorConfig
            //     .idleMode(IdleMode.kBrake)
            //     .smartCurrentLimit(50)
            //     .follow(ClimberConstants.CLIMBERMOTORLEFTID, ClimberConstants.RIGHTMOTORINVERTEDFROMLEFT);
        }
    }
}
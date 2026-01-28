package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;



public final class Configs {
    public static final class ShooterConfigs {
        public static final SparkMaxConfig shooterMotorConfig = new SparkMaxConfig();

        static {
            shooterMotorConfig
                .idleMode(IdleMode.kCoast);
        }
    }

    public static final class IntakeConfigs {
        public static final SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();

        static {
            intakeMotorConfig
                .idleMode(IdleMode.kCoast);
        }
    }
}
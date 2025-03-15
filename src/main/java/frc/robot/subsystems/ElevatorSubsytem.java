package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsytem extends SubsystemBase {
    public final SparkMax kLeftSparkMax = new SparkMax(Constants.ElevatorConstants.LeftMotorPort, SparkLowLevel.MotorType.kBrushless);
    public final SparkMax kRightSparkMax = new SparkMax(Constants.ElevatorConstants.RightMotorPort, SparkLowLevel.MotorType.kBrushless);
    public SparkMaxConfig config = new SparkMaxConfig();
    public final RelativeEncoder primaryEncoder = kLeftSparkMax.getEncoder();
    public final RelativeEncoder secondaryEncoder = kRightSparkMax.getEncoder(); //TODO: If I don't find a use for this by 3/20, remove it

    public void ElevatorSubsytem() {
        config
                .inverted(true)
                .idleMode(SparkBaseConfig.IdleMode.kBrake);
        kLeftSparkMax.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        config
                .inverted(false);
        kRightSparkMax.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }
    public  Command MoveElevatorWithStick(double speed) {
        return run( () -> {
            if (speed > 0.07) {
                kLeftSparkMax.set(speed);
                kRightSparkMax.set(speed);
            }
            else {
                kLeftSparkMax.set(0);
                kRightSparkMax.set(0);
            }
        });

    }
}

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    final SparkMax kRotateSparkMax = new SparkMax(Constants.IntakeConstants.rotationMotorPort,  SparkLowLevel.MotorType.kBrushless);
    final SparkMax kIntakeSparkMax = new SparkMax(Constants.IntakeConstants.intakeMotorPort, SparkLowLevel.MotorType.kBrushless);
    final RelativeEncoder kRotationNEOEncoder =  kRotateSparkMax.getEncoder();
    public IntakeSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config
                .idleMode(SparkBaseConfig.IdleMode.kCoast);
        kRotateSparkMax.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        kIntakeSparkMax.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        kRotationNEOEncoder.setPosition(0.0);
    }
    public Command flipDownIntakeToMax(double speed) {
        return run( () -> {
            while(kRotationNEOEncoder.getPosition() > Constants.IntakeConstants.IntakeStopPositionRotations) {
                kRotateSparkMax.set(speed * -1);
            }
        });
    }
    public Command flipUpIntakeToMax(double speed) {
        return run( () -> {
            while(kRotationNEOEncoder.getPosition() < 0) {
                kRotateSparkMax.set(speed);
            }
        });
    }
    public Command stickDriveIntake(double speed, boolean inverted) {
        return run( () -> {
            double adjustedSpeed = speed * -1;
            if(inverted){
                adjustedSpeed = speed * -1;
            } else if(!inverted){
                adjustedSpeed = speed;
            } else {
                adjustedSpeed = speed;
            }
            kRotateSparkMax.set(adjustedSpeed);
        });
    }
}

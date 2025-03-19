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

/**
 * @author Jonathan Weeks
 */
public class IntakeSubsystem extends SubsystemBase {
    final SparkMax kRotateSparkMax = new SparkMax(Constants.IntakeConstants.rotationMotorPort, SparkLowLevel.MotorType.kBrushless);
    final SparkMax kIntakeSparkMax = new SparkMax(Constants.IntakeConstants.intakeMotorPort, SparkLowLevel.MotorType.kBrushless);
    final RelativeEncoder kRotationNEOEncoder = kRotateSparkMax.getEncoder();

    public IntakeSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config
                .idleMode(SparkBaseConfig.IdleMode.kCoast);
        kRotateSparkMax.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        kIntakeSparkMax.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        kRotationNEOEncoder.setPosition(0.0);
    }

    public SparkMax getIntakeSparkMax() {
        return kIntakeSparkMax;
    }

    /**
     * Creates a command to lower the intake to its maximum position found in ({@link Constants.IntakeConstants})
     * by rotating the intake until it reaches the stop position.
     *
     * @param speed The speed at which to rotate the intake.
     * @return A command to lower the intake.
     */
    public Command flipDownIntakeToMax(double speed) {
        return run(() -> {
            while (kRotationNEOEncoder.getPosition() > Constants.IntakeConstants.IntakeStopPositionRotations) {
                kRotateSparkMax.set(speed * -1);
            }
        });
    }

    /**
     * Creates a command to raise the intake to its maximum position by rotating in reverse until it reaches the top position.
     *
     * @param speed The speed at which to rotate the intake.
     * @return A command to raise the intake.
     */
    public Command flipUpIntakeToMax(double speed) {
        return run(() -> {
            while (kRotationNEOEncoder.getPosition() < 0) {
                kRotateSparkMax.set(speed);
            }
        });
    }

    /**
     * Creates a command to manually rotate the intake based on input speed and direction.
     *
     * @param speed    The speed at which to rotate the intake.
     * @param inverted True if the rotation direction should be inverted.
     * @return A command to manually rotate the intake.
     */
    public Command stickDriveIntake(double speed, boolean inverted) {
        return run(() -> {
            double adjustedSpeed = speed * (inverted ? -1 : 1);
            kRotateSparkMax.set(adjustedSpeed);
        });
    }

}

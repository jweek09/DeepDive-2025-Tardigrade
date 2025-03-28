package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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
    final PIDController kPIDController = new PIDController(.3, 0, 0);

    boolean override = false;
    // I'm bad at naming things, so note that this speed is different from stickDriveIntake's speed. That one is local,
    // this is global, they don't meet.
    //TODO: refactor speed in intake subsystem so global/local speed have different names
    double speed = 0;
    double targetPosition = 0;

    public IntakeSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config
                .idleMode(SparkBaseConfig.IdleMode.kBrake);
        kRotateSparkMax.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        kIntakeSparkMax.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        kRotationNEOEncoder.setPosition(0.0);
        kPIDController.setSetpoint(targetPosition);
    }

    @Override
    public void periodic() {
        if (!override) {
            kPIDController.setSetpoint(targetPosition);
            speed = kPIDController.calculate(kRotationNEOEncoder.getPosition());
            kRotateSparkMax.set(MathUtil.clamp(speed, -.3,.3));
        }
    }

    public enum IntakeDirection {
        IN, OUT
    }

    public Command runIntake(IntakeDirection direction) {
        return run(() -> {
            int sign = (direction == IntakeDirection.IN) ? 1 : -1;
            kIntakeSparkMax.set(Constants.IntakeConstants.IntakeChoraleSpeed * sign);
        });
    }

    public Command stopIntake() {
        return run(() -> {
            kIntakeSparkMax.set(0);
        });
    }

    public Command flipDownIntake() {
        return run(() -> {
            targetPosition = Constants.IntakeConstants.IntakeStopPositionRotations;
        });
    }

    public Command flipUpIntake() {
        return run(() -> {
            targetPosition = 0.03;
        });
    }

    public Command intakeToL1() {
        return run(() -> {
            targetPosition = Constants.IntakeConstants.IntakeL1Rotations;
        });
    }

    public Command intakeToL2L3() {
        return run(() -> {
            targetPosition = Constants.IntakeConstants.IntakeL2L3Rotations;
        });
    }

    public Command intakeToPlayer() {
        return run( () -> {
            targetPosition = Constants.IntakeConstants.IntakePlayerRotations;
        });
    }

    public Command setPointManual(double setpoint) {
        return run ( () -> {
            targetPosition = setpoint;
        });
    }

}

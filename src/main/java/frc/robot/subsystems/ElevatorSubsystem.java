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

import java.util.function.DoubleSupplier;

/**
 * @author Jonathan Weeks
 */
public class ElevatorSubsystem extends SubsystemBase {
    final SparkMax kRightSparkMax = new SparkMax(Constants.ElevatorConstants.RightMotorPort, SparkLowLevel.MotorType.kBrushless);
    final SparkMax kLeftSparkMax = new SparkMax(Constants.ElevatorConstants.LeftMotorPort, SparkLowLevel.MotorType.kBrushless);
    final RelativeEncoder kPrimaryNEOEncoder = kRightSparkMax.getEncoder();
    final PIDController kPIDController = new PIDController(.2, 0, 0);

    boolean override = false;

    // I'm bad at naming things, so note that this speed is different from stickDriveIntake's speed. That one is local,
    // this is global, they don't meet.
    double speed = 0;
    double targetPosition = 0;

    public ElevatorSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                .inverted(true);
        kLeftSparkMax.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        config
                .inverted(false);
        kRightSparkMax.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    @Override
    public void periodic(){
        if (!override){
            kPIDController.setSetpoint(targetPosition);
            speed = kPIDController.calculate(kPrimaryNEOEncoder.getPosition());
            kRightSparkMax.set(MathUtil.clamp(speed, -.3, .3));
            kLeftSparkMax.set(MathUtil.clamp(speed, -.3, .3));
        }
    }

    public Command stickDriveIntake(DoubleSupplier speed) {
        if (override) {
            return run( () -> {
                kRightSparkMax.set(speed.getAsDouble());
                kLeftSparkMax.set(speed.getAsDouble());
            });
        } else {
            return run( () -> {});
        }
    }

    public Command toggleOverride() {
        return run( () -> {
            if (override) {
                override = false;
            } else if (!override) {
                override = true;
            }
        });
    }

    public Command ElevatorToHome() {
        return run( () -> {
            targetPosition = 1;
        });
    }

    public Command ElevatorToL1() {
        return run( () -> {
            targetPosition = Constants.ElevatorConstants.ElevatorL1Position;
        });
    }

    public Command ElevatorToL2() {
        return run( () -> {
            targetPosition = Constants.ElevatorConstants.ElevatorL2Position;
        });
    }

    public Command ElevatorToL3() {
        return run( () -> {
            targetPosition = Constants.ElevatorConstants.ElevatorL3Position;
        });
    }

    public Command setPointManual(double setpoint) {
        return run ( () -> {
            targetPosition = setpoint;
        });
    }

}

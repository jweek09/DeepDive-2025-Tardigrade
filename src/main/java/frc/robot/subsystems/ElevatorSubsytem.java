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

public class ElevatorSubsytem extends SubsystemBase {
    final SparkMax kLeftSparkMax = new SparkMax(Constants.ElevatorConstants.LeftMotorPort, SparkLowLevel.MotorType.kBrushless);
    final SparkMax kRightSparkMax = new SparkMax(Constants.ElevatorConstants.RightMotorPort, SparkLowLevel.MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    final RelativeEncoder primaryEncoder = kLeftSparkMax.getEncoder();
    final RelativeEncoder secondaryEncoder = kRightSparkMax.getEncoder(); //TODO: If I don't find a use for this by 3/20, remove it
    final PIDController pidController = new PIDController(2, 0, 0);

    public ElevatorSubsytem() {
        System.out.println("STARTING ELEVATOR SUBSYSTEM - Elevator");
        config
                .inverted(true)
                .idleMode(SparkBaseConfig.IdleMode.kBrake);
        kLeftSparkMax.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        config
                .inverted(false);
        kRightSparkMax.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        System.out.println("HELLO WORLD - Elevator");
        primaryEncoder.setPosition(0);
        secondaryEncoder.setPosition(0);
        kLeftSparkMax.set(0);
        kRightSparkMax.set(0);
    }

    /**
     *
     * @param speed Determines the speed to run the elevator at
     * @param inverted Determines whether the input should be inverted, does not determine direction like in the intake
     * @return Runs the command that drives the elevator
     */
    public Command MoveElevatorWithStick(DoubleSupplier speed, boolean inverted) {
        return run( () -> {
            System.out.println(kLeftSparkMax.getOutputCurrent());
            System.out.print("---" + kRightSparkMax.getOutputCurrent());
            double adjustedSpeed = MathUtil.clamp(speed.getAsDouble() * (inverted ? -1 : 1), -.03, 0.3);
            adjustedSpeed = MathUtil.applyDeadband(adjustedSpeed, 0.05);
            kLeftSparkMax.set(adjustedSpeed);
            kRightSparkMax.set(adjustedSpeed);
        });
    }
}

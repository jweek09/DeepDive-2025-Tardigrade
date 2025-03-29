package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants.SwerveConstants.ModuleConstants;

import static frc.robot.Constants.SwerveConstants.ModuleConstants.*;

public class SwerveModule implements Sendable {
    private final RelativeEncoder drivingEncoder;
    private final RelativeEncoder turningEncoder;

    private final SparkClosedLoopController drivingPIDController;
    private final SparkClosedLoopController turningPIDController;

    private final CANcoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;
    /** If reading the CANcoder is unsuccessful, this is how many times it will retry */
    private final int maximumCANcoderReadRetries = 5;
    /** Wait time for status frames to show up. */
    public static double STATUS_TIMEOUT_SECONDS = 0.02;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
                        int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
        // With REVLib 2025, Spark products are now configured via the Spark driveConfig objects
        final SparkMax drivingSparkMax;
        final SparkMax turningSparkMax;

        final SparkMaxConfig driveConfig = new SparkMaxConfig();
        final SparkMaxConfig turnConfig = new SparkMaxConfig();

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANcoder(absoluteEncoderId);

        drivingSparkMax = new SparkMax(driveMotorId, SparkLowLevel.MotorType.kBrushless);
        turningSparkMax = new SparkMax(turningMotorId, SparkLowLevel.MotorType.kBrushless);

        drivingSparkMax.clearFaults();
        turningSparkMax.clearFaults();

        drivingEncoder = drivingSparkMax.getEncoder();
        turningEncoder = turningSparkMax.getEncoder();
        drivingPIDController = drivingSparkMax.getClosedLoopController();
        turningPIDController = turningSparkMax.getClosedLoopController();

        driveConfig //configuring drivingSparkMax
                .inverted(driveMotorReversed) // Invert motors if driveMotorReversed is true
                .smartCurrentLimit(ModuleConstants.drivingMotorCurrentLimit)
                .idleMode(ModuleConstants.drivingMotorIdleMode)
                .closedLoopRampRate(DrivingRampRate);
        driveConfig.closedLoop
                .outputRange(ModuleConstants.drivingMinOutput , ModuleConstants.turningMaxOutput)
                .pid(drivingP, drivingI, drivingD)
                .velocityFF(drivingFF)
                .outputRange(ModuleConstants.drivingMinOutput , ModuleConstants.drivingMaxOutput);
        driveConfig.encoder
                .positionConversionFactor(ModuleConstants.driveEncoderRotToMeter)
                .velocityConversionFactor(ModuleConstants.driveEncoderRPMToMeterPerSec);

        // Save the SPARK MAX configurations after a power cycle by setting PersistMode to kPersistParameters
        // Reset undefined parameters to defaults by setting ResetMode to kResetSafeParameters
        drivingSparkMax.configure(driveConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        turnConfig
                .inverted(turningMotorReversed)
                .smartCurrentLimit(ModuleConstants.turningMotorCurrentLimit)
                .idleMode(ModuleConstants.turningMotorIdleMode);
        turnConfig.closedLoop
                .outputRange(ModuleConstants.turningMinOutput , ModuleConstants.turningMaxOutput)
                .pid(turningP, turningI, turningD)
                .velocityFF(turningFF)
                .positionWrappingEnabled(true)
                .positionWrappingMinInput(0)
                .positionWrappingMaxInput(2 * Math.PI);

        turnConfig.encoder
                .positionConversionFactor(ModuleConstants.turningEncoderRotToRad)
                .velocityConversionFactor(ModuleConstants.turningEncoderRPMToRadPerSec);
        turningSparkMax.configure(turnConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        new Thread(() -> { // Don't block anything else while sleeping
            try {
                Thread.sleep(500); // Gives the absolute encoders half-a-second to get started
                                        // TODO: See if this actually solves the problem of wheels being randomly skewed after starting
                resetEncoders();
            } catch (Exception e) {
                System.err.println(
                        "Failed to calibrate " + turningMotorId + " turning motor from absolute encoder. " +
                                "Something went wrong while sleeping the thread: \n\t" + e);
            }
        }).start();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveModule");

        builder.addDoubleProperty(
                "Raw Absolute Encoder Position",
                this::getRawAbsoluteEncoderRad,
                null);
        builder.addDoubleProperty(
                "Adjusted Absolute Encoder Position",
                () -> getAbsoluteEncoderRad(false),
                null);
        builder.addDoubleProperty("Drive Motor Position", this::getDrivePosition, null);
        builder.addDoubleProperty("Drive Motor Velocity", this::getDriveVelocity, null);
        builder.addDoubleProperty("Turn Motor Position", this::getTurningPosition, null);
        builder.addDoubleProperty("Turn Motor Velocity", this::getTurningVelocity, null);
    }

    public double getDrivePosition() {
        return drivingEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return drivingEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    /** Gets the raw, unadjusted absolute encoder rotation from the CANcoder cache */
    public double getRawAbsoluteEncoderRad() {
        StatusSignal<Angle> rotations = (absoluteEncoder.getAbsolutePosition());

        return (rotations.getValueAsDouble() / 360) // The value returned by the sensor is in rotations
                * (2 * Math.PI); // Convert to radians
    }

    /** Gets the absolute encoder rotation from the CANcoder
     * @param shouldRetry If true, the function will try up to
     * {@link SwerveModule#maximumCANcoderReadRetries maximumCANcoderReadRetries} times
     * to get a successful read from the sensor. Otherwise, it will just try to read immediately from the cached value*/
    public double getAbsoluteEncoderRad(boolean shouldRetry) {
        StatusSignal<Angle> rotations = absoluteEncoder.getAbsolutePosition();

        if (shouldRetry) {
            // Taken from democat's library.
            // Source:
            // https://github.com/democat3457/swerve-lib/blob/7c03126b8c22f23a501b2c2742f9d173a5bcbc40/src/main/java/com/swervedrivespecialties/swervelib/ctre/CanCoderFactoryBuilder.java#L51-L74
            for (int i = 0; i < maximumCANcoderReadRetries; i++) {
                if (rotations.getStatus() == StatusCode.OK) {
                    break;
                }
                rotations = rotations.waitForUpdate(STATUS_TIMEOUT_SECONDS);
            }
        }

        double v = (rotations.getValueAsDouble() / 360) // The value returned by the sensor is in rotations
                * (2 * Math.PI) // We want radians
                - (absoluteEncoderOffsetRad
                * (absoluteEncoderReversed ? -1 : 1));
        return v; // And we will invert, depending on the reversed state
    }

    /** Resets drive encoder to 0, and turning encoder to the reading of the absolute encoder */
    public void resetEncoders() {
        drivingEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad(true));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                getDrivePosition(),
                Rotation2d.fromRadians(getTurningPosition())
        );
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle); // Ensures the wheel never has to move more than 90°
        drivingPIDController.setReference(state.speedMetersPerSecond, SparkBase.ControlType.kVelocity);
        turningPIDController.setReference(state.angle.getRadians(), SparkBase.ControlType.kPosition);


    }

    /** Stops motors, telling drive motor to go to a velocity of 0 and the turning motor to hold its current rotation */
    public void stop() {
        drivingPIDController.setReference(0, SparkBase.ControlType.kVelocity);
        turningPIDController.setReference(getState().angle.getRadians(), SparkBase.ControlType.kPosition);
    }

}


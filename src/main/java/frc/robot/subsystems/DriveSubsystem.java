package frc.robot.subsystems;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import org.json.simple.parser.ParseException;

import java.io.IOException;


public class DriveSubsystem extends SubsystemBase {

    private final Field2d field = new Field2d();

    private final SwerveModule frontLeft = new SwerveModule(
            SwerveConstants.PortConstants.frontLeftDriveMotorPort,
            SwerveConstants.PortConstants.frontLeftTurningMotorPort,
            SwerveConstants.PhysicalConstants.frontLeftDriveEncoderReversed,
            SwerveConstants.PhysicalConstants.frontLeftTurningEncoderReversed,
            SwerveConstants.PortConstants.frontLeftDriveAbsoluteEncoderPort,
            SwerveConstants.PhysicalConstants.frontLeftDriveAbsoluteEncoderOffsetRad,
            SwerveConstants.PhysicalConstants.frontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            SwerveConstants.PortConstants.frontRightDriveMotorPort,
            SwerveConstants.PortConstants.frontRightTurningMotorPort,
            SwerveConstants.PhysicalConstants.frontRightDriveEncoderReversed,
            SwerveConstants.PhysicalConstants.frontRightTurningEncoderReversed,
            SwerveConstants.PortConstants.frontRightDriveAbsoluteEncoderPort,
            SwerveConstants.PhysicalConstants.frontRightDriveAbsoluteEncoderOffsetRad,
            SwerveConstants.PhysicalConstants.frontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            SwerveConstants.PortConstants.backLeftDriveMotorPort,
            SwerveConstants.PortConstants.backLeftTurningMotorPort,
            SwerveConstants.PhysicalConstants.backLeftDriveEncoderReversed,
            SwerveConstants.PhysicalConstants.backLeftTurningEncoderReversed,
            SwerveConstants.PortConstants.backLeftDriveAbsoluteEncoderPort,
            SwerveConstants.PhysicalConstants.backLeftDriveAbsoluteEncoderOffsetRad,
            SwerveConstants.PhysicalConstants.backLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            SwerveConstants.PortConstants.backRightDriveMotorPort,
            SwerveConstants.PortConstants.backRightTurningMotorPort,
            SwerveConstants.PhysicalConstants.backRightDriveEncoderReversed,
            SwerveConstants.PhysicalConstants.backRightTurningEncoderReversed,
            SwerveConstants.PortConstants.backRightDriveAbsoluteEncoderPort,
            SwerveConstants.PhysicalConstants.backRightDriveAbsoluteEncoderOffsetRad,
            SwerveConstants.PhysicalConstants.backRightDriveAbsoluteEncoderReversed);

    private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

    private final SwerveDriveOdometry poseEstimator = new SwerveDriveOdometry(
            SwerveConstants.swerveDriveKinematics,
            new Rotation2d(0),
            new SwerveModulePosition[] {
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    backLeft.getPosition(),
                    backRight.getPosition()
            });

    // With eager singleton initialization, any static variables/fields used in the 
    // constructor must appear before the "INSTANCE" variable so that they are initialized 
    // before the constructor is called when the "INSTANCE" variable initializes.

    /**
     * The Singleton instance of this DriveSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static DriveSubsystem INSTANCE = new DriveSubsystem();

    /**
     * Returns the Singleton instance of this DriveSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code DriveSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static DriveSubsystem getInstance() {
        return INSTANCE;
    }

    /**
     * Gets a PathPlanner path follower.
     * Events, if registered elsewhere using {@link NamedCommands}, will be run.
     *
     * @param pathName The PathPlanner path name, as configured in the configuration
     * @param setOdomToStart If true, will set the odometry to the start of the path when this command is initialized
     * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command
     */
    public Command getPathPlannerFollowCommand(String pathName, boolean setOdomToStart) {
        // Loads the path from the GUI name given
        PathPlannerPath path = null;
        try {
            path = PathPlannerPath.fromPathFile(pathName);
        } catch (IOException e) {
            throw new RuntimeException(e);
        } catch (ParseException e) {
            throw new RuntimeException(e);
        }

        if (setOdomToStart) {
            resetPose(new Pose2d(path.getPoint(0).position, getGyroRotation2d()));
        }

        // Creates a path following command using AutoBuilder. This will execute any named commands when running
        return AutoBuilder.followPath(path);
    }

    public Command pathfindToPosition(Pose2d position) {
        return AutoBuilder.pathfindToPose(position, new PathConstraints(
                SwerveConstants.AutoConstants.maxSpeedMetersPerSecond,
                SwerveConstants.AutoConstants.maxAccelerationMetersPerSecondSquared,
                SwerveConstants.AutoConstants.maxAngularSpeedRadiansPerSecond,
                SwerveConstants.AutoConstants.maxAngularAccelerationRadiansPerSecondSquared));
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return SwerveConstants.swerveDriveKinematics.toChassisSpeeds(
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
        );
    }

    public void zeroHeading() {
        gyro.reset();
    }

    /** Gets the rotation reported by the heading in degrees
     * @return The reported angle, in degrees */
    private double getHeading() {
        return -Math.IEEEremainder(gyro.getAngle(), 360);
    }

    private Rotation2d getGyroRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Rotation2d getOdometryHeading() {
        return getPose().getRotation();
    }

    public Pose2d getPose() {
        return poseEstimator.getPoseMeters();
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(getGyroRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                }, pose);
        field.setRobotPose(poseEstimator.getPoseMeters());
    }

    public void resetWheelEncoders() {
        var pose = poseEstimator.getPoseMeters(); // Save it so the pose isn't reset
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
        resetPose(pose);
    }

    @Override
    public void periodic() {
        poseEstimator.update(getGyroRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                });

        field.setRobotPose(poseEstimator.getPoseMeters());
    }

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        setModuleStates(SwerveConstants.swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds));
    }

    /** Stops all modules, setting their velocity to 0 and instructing them to hold their current rotation */
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.PhysicalConstants.physicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    /**
     * Creates a new instance of this DriveSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    public DriveSubsystem() {

        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
//        SmartDashboard.putData("NavX Gyroscope", gyro);
//        SmartDashboard.putData("Front Left Swerve Module", frontLeft);
//        SmartDashboard.putData("Front Right Swerve Module", frontRight);
//        SmartDashboard.putData("Back Left Swerve Module", backLeft);
//        SmartDashboard.putData("Back Right Swerve Module", backRight);

        SmartDashboard.putData("Field", field);

        new Thread(() -> { // Don't block anything else while sleeping
            try {
                Thread.sleep(1000); // Waits for the gyro to boot up before resetting the heading
                zeroHeading();
            } catch (Exception e) {
                System.err.println(
                        "Failed to zero gyro heading. Something went wrong while sleeping the thread: \n\t" + e);
            }
        }).start();

        RobotConfig config = null;
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            throw new RuntimeException("Robot not configured in PathPlanner - Unable to load config in DriveSubsystem");
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );}
}


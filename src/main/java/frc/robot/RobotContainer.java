// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();
    
    private DoubleSupplier defaultLauncherSpeed;
    private final BooleanSupplier isRedAlliance = () ->
            DriverStation.getAlliance().filter(value -> value == Alliance.Red).isPresent();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController driverController =
            new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

    private boolean teleopDriveSpeedReduced = false;

    private final CommandXboxController operatorController =
            new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

    private final IntakeSubsystem kIntakeSubsystem = new IntakeSubsystem();
    private final ElevatorSubsystem kElevatorSubsystem = new ElevatorSubsystem();

    //private final SendableChooser<Command> autonomousCommand;
    private GenericEntry autonomousDelayTime;

    private SendableChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        registerAutonomousCommands();

        configureDashboard();

        autoChooser = AutoBuilder.buildAutoChooser();
        Shuffleboard.getTab("Autochooser")
                .add("Autonomous Mode", autoChooser)
                .withWidget(BuiltInWidgets.kComboBoxChooser);

        // Configure the trigger bindings
        configureBindings();
    }

    private void registerAutonomousCommands() {
        NamedCommands.registerCommand("Intake To Home", kIntakeSubsystem.flipUpIntake());
        NamedCommands.registerCommand("Intake To L1", kIntakeSubsystem.intakeToL1());
        NamedCommands.registerCommand("Intake To L2/L3", kIntakeSubsystem.intakeToL2L3());
        NamedCommands.registerCommand("Intake To Player", kIntakeSubsystem.intakeToPlayer());
        NamedCommands.registerCommand("Elevator To Home", kElevatorSubsystem.ElevatorToHome());
        NamedCommands.registerCommand("Elevator To L1", kElevatorSubsystem.ElevatorToL1());
        NamedCommands.registerCommand("Elevator To L2", kElevatorSubsystem.ElevatorToL2());
        NamedCommands.registerCommand("Elevator To L3", kElevatorSubsystem.ElevatorToL3());
        NamedCommands.registerCommand("Combo To Home", new ParallelCommandGroup(
                kElevatorSubsystem.ElevatorToHome(), kIntakeSubsystem.flipUpIntake()
        ));
        NamedCommands.registerCommand("Combo To L1", new ParallelCommandGroup(
                kElevatorSubsystem.ElevatorToL1(), kIntakeSubsystem.intakeToL1()
        ));
        NamedCommands.registerCommand("Combo To L2", new ParallelCommandGroup(
                kElevatorSubsystem.ElevatorToL2(), kIntakeSubsystem.intakeToL2L3()
        ));
        NamedCommands.registerCommand("Combo To L3", new ParallelCommandGroup(
                kElevatorSubsystem.ElevatorToL3(), kIntakeSubsystem.intakeToL2L3()
        ));
        NamedCommands.registerCommand("Combo To Player", new ParallelCommandGroup(
                kElevatorSubsystem.ElevatorToL1(), kIntakeSubsystem.intakeToPlayer()
        ));
        NamedCommands.registerCommand("Run Intake", kIntakeSubsystem.runIntake(IntakeSubsystem.IntakeDirection.IN));
        NamedCommands.registerCommand("Run Outtake", kIntakeSubsystem.runIntake(IntakeSubsystem.IntakeDirection.OUT));
        NamedCommands.registerCommand("Stop Intake", kIntakeSubsystem.stopIntake());
    }

    /**
     * Configures the Shuffleboard
     */
    private void configureDashboard() {
    }
    
    
    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        driveSubsystem.setDefaultCommand(
                driveSubsystem.driveRobotRelativeCommand(
                        driverController::getLeftY,
                        driverController::getLeftX,
                        driverController::getRightX
                )
        );

        driverController.rightBumper().whileTrue(
                driveSubsystem.driveRobotRelativeCommandSlow(
                        driverController::getLeftY,
                        driverController::getLeftX,
                        driverController::getRightX
                )
        );


        driverController.leftTrigger().whileTrue(kIntakeSubsystem.runIntake(IntakeSubsystem.IntakeDirection.IN)).onFalse(kIntakeSubsystem.stopIntake());
        driverController.rightTrigger().whileTrue(kIntakeSubsystem.runIntake(IntakeSubsystem.IntakeDirection.OUT)).onFalse(kIntakeSubsystem.stopIntake());

        operatorController.leftTrigger().whileTrue(kIntakeSubsystem.runIntake(IntakeSubsystem.IntakeDirection.IN)).onFalse(kIntakeSubsystem.stopIntake());
        operatorController.rightTrigger().whileTrue(kIntakeSubsystem.runIntake(IntakeSubsystem.IntakeDirection.OUT)).onFalse(kIntakeSubsystem.stopIntake());
        operatorController.rightBumper().onTrue(new ParallelCommandGroup(kElevatorSubsystem.ElevatorToL3(), kIntakeSubsystem.intakeToL2L3()));
        operatorController.leftBumper().onTrue(new ParallelCommandGroup(kElevatorSubsystem.ElevatorToL2(), kIntakeSubsystem.intakeToL2L3()));

        operatorController.y().onTrue(kIntakeSubsystem.flipUpIntake());
        operatorController.x().onTrue(kIntakeSubsystem.intakeToL1());
        operatorController.b().onTrue(kIntakeSubsystem.intakeToPlayer());
        operatorController.a().onTrue(kIntakeSubsystem.intakeToL2L3());

        operatorController.povUp().onTrue(kElevatorSubsystem.ElevatorToL3());
        operatorController.povRight().onTrue(kElevatorSubsystem.ElevatorToL2());
        operatorController.povLeft().onTrue(kElevatorSubsystem.ElevatorToL1());
        operatorController.povDown().onTrue(kElevatorSubsystem.ElevatorToHome());
    }
    public Command getAutonomousCommand() {
        //return Commands.waitSeconds(MathUtil.clamp(autonomousDelayTime.getDouble(0.0), 0, 15))
        //.andThen(autonomousCommand.getSelected());
        return autoChooser.getSelected();
    }
}

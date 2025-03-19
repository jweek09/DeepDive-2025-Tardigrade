package frc.robot.commands;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeChoraleWithSpeed extends Command {
    private final double speed;
    private final boolean direction;
    private final SparkMax intakeSparkmax;

    /**
     *
     * @param speed will determine the speed to run the intake
     * @param direction will determine whether to pull in a choral (true) or push out a choral (false)
     */
    public IntakeChoraleWithSpeed(double speed, boolean direction, SparkMax intakeSparkmax) {
        this.speed = speed;
        this.direction = direction;
        this.intakeSparkmax = intakeSparkmax;
    }
    @Override
    public void initialize() {
        intakeSparkmax.set(speed * (direction ? 1 : -1));
    }
    @Override
    public void end(boolean interrupted) {
        intakeSparkmax.set(0);
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb;


public class ClimbSetSpeed extends CommandBase {
    private final Climb climb;
    private double speed;

    public ClimbSetSpeed(Climb climb, double speed) {
        this.climb = climb;
        this.speed = speed;
        addRequirements(this.climb);
    }

    @Override
    public void initialize() {
        climb.setClimbSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

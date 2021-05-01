package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb;


public class ClimbSetHold extends CommandBase {
    private final Climb climb;

    public ClimbSetHold(Climb climb) {
        this.climb = climb;
        addRequirements(this.climb);
    }

    @Override
    public void initialize() {
        climb.setClimbHold();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

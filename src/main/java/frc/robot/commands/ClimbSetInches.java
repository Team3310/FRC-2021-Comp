package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb;


public class ClimbSetInches extends CommandBase {
    private final Climb climb;
    private double inches;

    public ClimbSetInches(Climb climb, double inches) {
       this.climb = climb;
       this.inches = inches;
        addRequirements(this.climb);
    }

    @Override
    public void initialize() {
    climb.setClimbMotionMagicPositionAbsolute(inches);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }
}

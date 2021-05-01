package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;


public class ShooterSetLimelightTrackMode extends CommandBase {
    private final Shooter shooter;

    public ShooterSetLimelightTrackMode(Shooter shooter) {
        this.shooter = shooter;
//        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setShooterLimelightTrackMode();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

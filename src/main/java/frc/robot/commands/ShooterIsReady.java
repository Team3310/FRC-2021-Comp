package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;


public class ShooterIsReady extends CommandBase {
    private final Shooter shooter;

    public ShooterIsReady(Shooter shooter) {
        this.shooter = shooter;
    }

    @Override
    public boolean isFinished() {
        return shooter.isReady();
    }
}

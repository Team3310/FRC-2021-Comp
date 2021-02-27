package frc.robot.commands;

import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;


public class HoodSetAngleFromDist extends ExtraTimeoutCommand {
    private final Shooter shooter;
    private final Limelight limelight;
    private double shooterDistanceToGoal;

    public HoodSetAngleFromDist(Shooter shooter, Limelight limelight) {
        this.shooter = shooter;
        this.limelight = limelight;
    }

    @Override
    public void initialize() {
        shooterDistanceToGoal = limelight.getDistanceFromTargetInches();
        shooter.setHoodAngleBasedOnDistance(shooterDistanceToGoal);
        resetExtraOneTimer();
        startExtraOneTimeout(0.1);
    }

    @Override
    public boolean isFinished() {
        if (isExtraOneTimedOut() && shooter.hasFinishedHoodTrajectory()) {
            return true;
        }
        return false;
    }
}
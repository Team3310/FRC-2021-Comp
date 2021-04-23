package frc.robot.commands;

import frc.robot.subsystems.Shooter;


public class HoodSetAngleDistance extends ExtraTimeoutCommand {
    private final Shooter shooter;
    private double angle;

    public HoodSetAngleDistance(Shooter shooter) {
        this.shooter = shooter;
    }

    @Override
    public void initialize() {
        resetExtraOneTimer();
        startExtraOneTimeout(0.1);
    }

    @Override
    public boolean isFinished() {
        angle = shooter.getHoodAngleFromDistance();
        shooter.setHoodMotionMagicPositionAbsolute(angle);
//        if (isExtraOneTimedOut() && shooter.hasFinishedHoodTrajectory()) {
//            return true;
//        }
        return false;
    }
}

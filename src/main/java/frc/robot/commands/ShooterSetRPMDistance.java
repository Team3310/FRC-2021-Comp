package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class ShooterSetRPMDistance extends CommandBase {
    private final Shooter shooter;
    private double rpm;
    private double lastRPM;

    public ShooterSetRPMDistance(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {

    }

    @Override
    public boolean isFinished() {
//        boolean isMainUpToSpeed = Util.epsilonEquals(shooter.getMainRPM(), rpm, Constants.SHOOTER_MAIN_RPM_EPSILON);
//        boolean isKickerUpToSpeed = Util.epsilonEquals(shooter.getKickerRPM(), rpm, Constants.SHOOTER_KICKER_RPM_EPSILON);
//        return isMainUpToSpeed && isKickerUpToSpeed;
        if (Limelight.getInstance().isOnTarget()) {
            rpm = shooter.getRPMFromDistance();
            lastRPM = rpm;
        } else {
            rpm = lastRPM;
        }

        shooter.setMainRPM(rpm);
        shooter.setKickerRPM(rpm);
        return false;
    }
}

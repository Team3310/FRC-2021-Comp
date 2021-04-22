package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.Util;


public class ShooterSetRPMDistance extends CommandBase {
    private final Shooter shooter;
    private double rpm;


    public ShooterSetRPMDistance(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        rpm = shooter.getRPMFromDistance();
        shooter.setMainRPM(rpm);
        shooter.setKickerRPM(rpm);
    }

    @Override
    public boolean isFinished() {
        boolean isMainUpToSpeed = Util.epsilonEquals(shooter.getMainRPM(), rpm, Constants.SHOOTER_MAIN_RPM_EPSILON);
        boolean isKickerUpToSpeed = Util.epsilonEquals(shooter.getKickerRPM(), rpm, Constants.SHOOTER_KICKER_RPM_EPSILON);
        return isMainUpToSpeed && isKickerUpToSpeed;
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.utilities.Util;


public class ShooterSetEverything extends CommandBase {
    private final Shooter shooter;
    private double rpm;
    private double lastRPM;
    private double hoodAngle;
    private double lastHoodAngle;


    public ShooterSetEverything(Shooter shooter) {
        this.shooter = shooter;
//        addRequirements(shooter);
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
            hoodAngle = shooter.getHoodAngleFromDistance();
            shooter.setHoodMotionMagicPositionAbsolute(hoodAngle);
            lastHoodAngle = hoodAngle;
        } else {
            rpm = lastRPM;
            hoodAngle = lastHoodAngle;
        }

        shooter.setMainRPM(rpm);
        shooter.setKickerRPM(rpm);
        shooter.setHoodMotionMagicPositionAbsolute(hoodAngle);
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Everything interrupted = " + interrupted);
    }
}

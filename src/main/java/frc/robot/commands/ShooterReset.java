package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class ShooterReset extends SequentialCommandGroup {

    public ShooterReset(Shooter shooter, Magazine magazine, Limelight limelight, Turret turret) {
        addCommands(
                new MagazineSetSpeed(magazine,0),
//                new ShooterSetSpeed(shooter, 0, 0),
                new ShooterIntakeSetSpeed(shooter, 0),
                new LimelightSetLED(limelight, Limelight.LightMode.OFF),
                new InstantCommand(() -> shooter.setShooterControlMode(Shooter.ShooterControlMode.MANUAL)),
                new HoodSetAngle(shooter,Constants.HOOD_RETRACT_HOME_POSITION_DEGREES),
                new ShooterSetReady(shooter,false),
//                new TurretAutoZero(turret),
//                new TurretSetAngle(turret,(-180 + turret.getLagAngle(Drive.getInstance()) -Drive.getInstance().getNormalizeGyro(Drive.getInstance()))),
                new TurretSetAngle(turret, Constants.TURRET_COMPETITION_HOME_POSITION_DEGREES),
                new ShooterSetReady(shooter,true)
            );
    }
}
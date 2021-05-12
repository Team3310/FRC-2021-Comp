package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.MagazineSetRPMRotations;
import frc.robot.commands.ShooterIntakeSetRPM;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;

public class ShooterShootAuto extends ParallelCommandGroup {

    public ShooterShootAuto(Shooter shooter, Magazine magazine, double magazineRotations) {
        addCommands(
                new ShooterIntakeSetRPM(shooter, Constants.SHOOTER_INTAKE_RPM),
                new MagazineSetRPMRotations(magazine, Constants.MAGAZINE_SHOOT_AUTO_RPM,
                        magazineRotations)
        );
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.*;


public class ScoreWhileDriving extends SequentialCommandGroup {

    public ScoreWhileDriving(Drive drive, Shooter shooter, Turret turret, Magazine magazine, Limelight limelight){
        addCommands(
                new ParallelCommandGroup(
                    new LimelightSetLED(limelight, Limelight.LightMode.ON),
                    new InstantCommand(()->limelight.setPipeline(Constants.LIMELIGHT_ALL_FIELD_PIPELINE)),
                    new TurretSetToTrackGoal(turret, turret.getLagAngle(drive)),
                    new HoodSetToTrackLimelightAngle(shooter),
                    new ShooterSetRPM(shooter, Constants.SHOOTER_MAIN_ALL_FIELD_RPM, Constants.SHOOTER_KICKER_ALL_FIELD_RPM)),
//                new MagazineSetRPM(magazine, Constants.MAGAZINE_SHOOT_RPM),
//                new ShooterIntakeSetRPM(shooter, Constants.SHOOTER_INTAKE_RPM),
                new ShooterReset(shooter,magazine,Limelight.getInstance(), turret)



        );

    }
}
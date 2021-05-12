package frc.robot.auto.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class ResetAllHomeAuto extends SequentialCommandGroup {

    public ResetAllHomeAuto(Drive drive, Turret turret, Magazine magazine, Shooter shooter, Climb climb, Pose2d startPose) {
        addCommands(
                new InstantCommand(()-> shooter.resetHoodHomePosition()),
                new InstantCommand(()-> climb.resetClimbEncoder()),
                new InstantCommand(()-> climb.setClimbMotionMagicPositionAbsolute(0)),
                new InstantCommand(()-> magazine.resetHomePosition()),
                new InstantCommand(()-> turret.resetHomePosition(Constants.TURRET_COMPETITION_HOME_POSITION_DEGREES)),
                new ResetOdometryAuto(startPose)
         );
    }
}
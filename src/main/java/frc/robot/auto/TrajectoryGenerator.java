package frc.robot.auto;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import java.util.List;

public class TrajectoryGenerator {
    private static final TrajectoryGenerator instance = new TrajectoryGenerator();

    public static TrajectoryGenerator getInstance() {
        return instance;
    }

    // Create a voltage constraint to ensure we don't accelerate too fast
    DifferentialDriveVoltageConstraint autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                    new SimpleMotorFeedforward(Constants.ksVolts,
                            Constants.kvVoltSecondsPerMeter,
                            Constants.kaVoltSecondsSquaredPerMeter),
                    Constants.kDriveKinematics,
                    10);

    // Create config for trajectory
    TrajectoryConfig forwardConfig =
            new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                    Constants.kMaxAccelerationMetersPerSecondSquared)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(Constants.kDriveKinematics)
                    // Apply the voltage constraint
                    .addConstraint(autoVoltageConstraint);

    TrajectoryConfig forwardMediumConfig =
            new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond * 1.2,
                    Constants.kMaxAccelerationMetersPerSecondSquared)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(Constants.kDriveKinematics)
                    // Apply the voltage constraint
                    .addConstraint(autoVoltageConstraint);

    TrajectoryConfig forwardMediumSlowConfig =
            new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond * .82,
                    Constants.kMaxAccelerationMetersPerSecondSquared)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(Constants.kDriveKinematics)
                    // Apply the voltage constraint
                    .addConstraint(autoVoltageConstraint);

    TrajectoryConfig forwardConfigSlow =
            new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond * 0.5,
                    Constants.kMaxAccelerationMetersPerSecondSquared)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(Constants.kDriveKinematics)
                    // Apply the voltage constraint
                    .addConstraint(autoVoltageConstraint);

    TrajectoryConfig forwardFastConfig =
            new TrajectoryConfig(Constants.kMaxFastSpeedMetersPerSecond,
                    Constants.kMaxAccelerationMetersPerSecondSquared)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(Constants.kDriveKinematics)
                    // Apply the voltage constraint
                    .addConstraint(autoVoltageConstraint);

    TrajectoryConfig reverseConfig =
            new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                    Constants.kMaxAccelerationMetersPerSecondSquared)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(Constants.kDriveKinematics)
                    // Apply the voltage constraint
                    .addConstraint(autoVoltageConstraint)
                    .setReversed(true);

    TrajectoryConfig reverseConfigSlow =
            new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond * 0.5,
                    Constants.kMaxAccelerationMetersPerSecondSquared)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(Constants.kDriveKinematics)
                    // Apply the voltage constraint
                    .addConstraint(autoVoltageConstraint);

    TrajectoryConfig reverseFastConfig =
            new TrajectoryConfig(Constants.kMaxFastSpeedMetersPerSecond,
                    Constants.kMaxFastAccelerationMetersPerSecondSquared)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(Constants.kDriveKinematics)
                    // Apply the voltage constraint
                    .addConstraint(autoVoltageConstraint)
                    .setReversed(true);


    public Trajectory getDriveStraight(){
            return edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0),
                            new Rotation2d(Units.radiansToDegrees(0))),
                    List.of(
                            new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(0))
                    ),
                    new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(0),
                            new Rotation2d(Units.radiansToDegrees(0))),
                    // Pass config
                    forwardConfig
            );
        }

        public Trajectory getDriveStraightReversed(){
            return edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(0),
                            new Rotation2d(Units.radiansToDegrees(0))),
                    List.of(
                            new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(0))
                    ),
                    new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0),
                            new Rotation2d(Units.radiansToDegrees(0))),
                    // Pass config
                    reverseConfig
            );
        }
        public Trajectory pathBRed(){
            return RobotContainer.loadPathTrajectory("paths/PathBRed.wpilib.json");
        }

        public Trajectory getStealStartToStealBallV2() {
            Trajectory stealStartToStealSpot;
            stealStartToStealSpot = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(147), Units.inchesToMeters(-245), new Rotation2d(0)),
                List.of(
                        new Translation2d(Units.inchesToMeters(201), Units.inchesToMeters(-264))
                ),
                new Pose2d(Units.inchesToMeters(271), Units.inchesToMeters(-303), Rotation2d.fromDegrees(-45.0)),
                // Pass config
                forwardFastConfig
            );
            return stealStartToStealSpot;
        }

        public Trajectory getStealBallToCenterShotV2() {
            Trajectory stealSpotToCenterShot;
            stealSpotToCenterShot = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(271), Units.inchesToMeters(-303), Rotation2d.fromDegrees(-45)),
                    List.of(
                            new Translation2d(Units.inchesToMeters(235), Units.inchesToMeters(-200))
                    ),
                    new Pose2d(Units.inchesToMeters(185), Units.inchesToMeters(-120), Rotation2d.fromDegrees(-90)),
                    // Pass config
                    reverseFastConfig
            );
            return stealSpotToCenterShot;
        }

    public Trajectory getFirstTwoBalls() {
        Trajectory FirstTwoBalls;
        FirstTwoBalls = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(136), Units.inchesToMeters(-60), new Rotation2d(Units.degreesToRadians(0))),
                List.of(
                        new Translation2d(Units.inchesToMeters(213.098), Units.inchesToMeters(-77.632))
                ),
                new Pose2d(Units.inchesToMeters(257.5), Units.inchesToMeters(-138), new Rotation2d(Units.degreesToRadians(-75))),
                forwardMediumConfig
        );
        return FirstTwoBalls;

    }

    public Trajectory getFirstFourBalls() {
        Trajectory FirstFourBalls;
        FirstFourBalls = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(136), Units.inchesToMeters(-95), new Rotation2d(Units.degreesToRadians(0))),
                List.of(
                        new Translation2d(Units.inchesToMeters(230), Units.inchesToMeters(-35))
                ),
                new Pose2d(Units.inchesToMeters(236), Units.inchesToMeters(-155), new Rotation2d(Units.degreesToRadians(-110))),
                forwardMediumSlowConfig
        );
        return FirstFourBalls;
    }

    public Trajectory reverseToShootFromFour() {
        Trajectory ShootFromFour;
        ShootFromFour = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(235), Units.inchesToMeters(-155), new Rotation2d(Units.degreesToRadians(-110))),
                List.of(
                        new Translation2d(Units.inchesToMeters(235), Units.inchesToMeters(-75))
                ),
                new Pose2d(Units.inchesToMeters(230), Units.inchesToMeters(-35), new Rotation2d(Units.degreesToRadians(-70))),
                reverseConfig
        );
        return ShootFromFour;
    }

    public Trajectory grab1Ball() {
        Trajectory grab1Ball;
        grab1Ball = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(230), Units.inchesToMeters(-35), new Rotation2d(Units.degreesToRadians(-70))),
                List.of(
                        new Translation2d(Units.inchesToMeters(220), Units.inchesToMeters(-95))
                ),
                new Pose2d(Units.inchesToMeters(220), Units.inchesToMeters(-100), new Rotation2d(Units.degreesToRadians(-70))),
                forwardConfig
        );
        return grab1Ball;
    }

    public Trajectory grab1BallToShoot() {
        Trajectory grab1BallToShoot;
        grab1BallToShoot = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(220), Units.inchesToMeters(-100), new Rotation2d(Units.degreesToRadians(-70))),
                List.of(
                        new Translation2d(Units.inchesToMeters(230), Units.inchesToMeters(-90))
                ),
                new Pose2d(Units.inchesToMeters(235), Units.inchesToMeters(-70), new Rotation2d(Units.degreesToRadians(-70))),
                reverseConfig
        );
        return grab1BallToShoot;
    }

    public Trajectory getFirst2BallsRend() {
        Trajectory First2BallsRend;
        First2BallsRend = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(136), Units.inchesToMeters(-95), new Rotation2d(Units.degreesToRadians(0))),
                List.of(
                        new Translation2d(Units.inchesToMeters(230), Units.inchesToMeters(-65))
                ),
                new Pose2d(Units.inchesToMeters(245), Units.inchesToMeters(-145), new Rotation2d(Units.degreesToRadians(-75))),
                forwardConfig
        );
        return First2BallsRend;
    }

    public Trajectory getToShootFromRend() {
        Trajectory ShootRend;
        ShootRend = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(245), Units.inchesToMeters(-145), new Rotation2d(Units.degreesToRadians(-75))),
                List.of(
                        new Translation2d(Units.inchesToMeters(238), Units.inchesToMeters(-100))
                ),
                new Pose2d(Units.inchesToMeters(230), Units.inchesToMeters(-72), new Rotation2d(Units.degreesToRadians(-75))),
                reverseConfig
        );
        return  ShootRend;
    }

    public Trajectory getRend3BallTrajectory() {
        Trajectory grabRend3;
        grabRend3 = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(230), Units.inchesToMeters(-72), new Rotation2d(Units.degreesToRadians(-75))),
                List.of(
                        new Translation2d(Units.inchesToMeters(241), Units.inchesToMeters(-115))
                ),
                new Pose2d(Units.inchesToMeters(260), Units.inchesToMeters(-159), new Rotation2d(Units.degreesToRadians(-95))),
                forwardConfig
        );
        return  grabRend3;
    }

    public Trajectory getRend3BallTrajectoryReverse() {
        Trajectory grabRend3Rev;
        grabRend3Rev = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(260), Units.inchesToMeters(-159), new Rotation2d(Units.degreesToRadians(-95))),
                List.of(
                        new Translation2d(Units.inchesToMeters(241), Units.inchesToMeters(-115))
                ),
                new Pose2d(Units.inchesToMeters(230), Units.inchesToMeters(-72), new Rotation2d(Units.degreesToRadians(-75))),
                reverseConfig
        );
        return  grabRend3Rev;
    }

    public Trajectory RendGetFour() {
        Trajectory RendGetFour;
        RendGetFour = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(230), Units.inchesToMeters(-72), new Rotation2d(Units.degreesToRadians(-75))),
                List.of(
                        new Translation2d(Units.inchesToMeters(300), Units.inchesToMeters(-100))
                ),
                new Pose2d(Units.inchesToMeters(350), Units.inchesToMeters(-115), new Rotation2d(Units.degreesToRadians(-205))),
                forwardConfig
        );
        return  RendGetFour;
    }

    public Trajectory GoStraightToGetFour() {
        Trajectory GetFour;
        GetFour = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(350), Units.inchesToMeters(-115), new Rotation2d(Units.degreesToRadians(-205))),
                List.of(
                        new Translation2d(Units.inchesToMeters(320), Units.inchesToMeters(-120))
                ),
                new Pose2d(Units.inchesToMeters(220), Units.inchesToMeters(-134), new Rotation2d(Units.degreesToRadians(-200))),
                forwardConfig
        );
        return  GetFour;
    }

    public Trajectory getFirstTwoBallsReversed() {
        Trajectory FirstTwoBallsReversed;
        FirstTwoBallsReversed = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(257.5), Units.inchesToMeters(-138), new Rotation2d(Units.degreesToRadians(-75))),
                List.of(
                        new Translation2d(Units.inchesToMeters(213.098), Units.inchesToMeters(-77.632))
                ),
                new Pose2d(Units.inchesToMeters(136), Units.inchesToMeters(-60), new Rotation2d(Units.degreesToRadians(0))),
                reverseFastConfig
        );
        return FirstTwoBallsReversed;
    }

    public Trajectory getFirstTwoBallsSteal() {
        Trajectory FirstTwoBalls;
        FirstTwoBalls = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(136), Units.inchesToMeters(-60), new Rotation2d(Units.degreesToRadians(0))),
                List.of(
                        new Translation2d(Units.inchesToMeters(213.098), Units.inchesToMeters(-77.632))
                ),
                new Pose2d(Units.inchesToMeters(285.5), Units.inchesToMeters(-159), new Rotation2d(Units.degreesToRadians(-75))),
                forwardFastConfig
        );
        return FirstTwoBalls;

    }
    public Trajectory getFirstTwoBallsReversedSteal() {
        Trajectory FirstTwoBallsReversed;
        FirstTwoBallsReversed = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(285.5), Units.inchesToMeters(-159), new Rotation2d(Units.degreesToRadians(-75))),
                List.of(
                        new Translation2d(Units.inchesToMeters(213.098), Units.inchesToMeters(-77.632))
                ),
                new Pose2d(Units.inchesToMeters(136), Units.inchesToMeters(-60), new Rotation2d(Units.degreesToRadians(-45))),
                reverseFastConfig
        );
        return FirstTwoBallsReversed;
    }

    public Trajectory goFromAutoLineToBarShot() {
        Trajectory goFromAutoLineToBarShot;
        goFromAutoLineToBarShot = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(136), Units.inchesToMeters(-60), new Rotation2d(Units.degreesToRadians(-45))),
                List.of(
                        new Translation2d(Units.inchesToMeters(151), Units.inchesToMeters(-98.7))
                ),
                new Pose2d(Units.inchesToMeters(189), Units.inchesToMeters(-155), new Rotation2d(Units.degreesToRadians(0))),
                forwardFastConfig
        );
        return goFromAutoLineToBarShot;
    }

    public Trajectory getTrench3Ball() {
        Trajectory Trench3Ball;
        Trench3Ball = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(136), Units.inchesToMeters(-60), new Rotation2d(Units.degreesToRadians(0))),
                List.of(
                        new Translation2d(Units.inchesToMeters(181.547), Units.inchesToMeters(-39.622)),
                        new Translation2d(Units.inchesToMeters(212.104), Units.inchesToMeters(-30.927))

                ),

                new Pose2d(Units.inchesToMeters(316.694), Units.inchesToMeters(-28.691), new Rotation2d(Units.degreesToRadians(0))),
                forwardConfig
        );
        return Trench3Ball;

    }

    public Trajectory getTrench5Ball() {
        Trajectory Trench5Ball;
        Trench5Ball = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(136), Units.inchesToMeters(-29.5), new Rotation2d(0)),
                List.of(
                        new Translation2d(Units.inchesToMeters(200.547), Units.inchesToMeters(-29.5))

                ),

                new Pose2d(Units.inchesToMeters(389), Units.inchesToMeters(-29.5), new Rotation2d(0)),
                forwardConfig
        );
        return Trench5Ball;

    }
    public Trajectory getToMediumFrom5Trench() {
        Trajectory getToMediumFrom5Trench;
        getToMediumFrom5Trench = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(389), Units.inchesToMeters(-29.5), new Rotation2d(Units.degreesToRadians(0))),
                List.of(
                        new Translation2d(Units.inchesToMeters(290.000), Units.inchesToMeters(-29.5))

                ),

                new Pose2d(Units.inchesToMeters(199.755), Units.inchesToMeters(-29.5), new Rotation2d(Units.degreesToRadians(0))),
                reverseFastConfig
        );
        return getToMediumFrom5Trench;

    }

    public Trajectory getTrench2Ball() {
        Trajectory Trench2Ball;
        Trench2Ball = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(136), Units.inchesToMeters(-29.5), new Rotation2d(0)),
                List.of(
                        new Translation2d(Units.inchesToMeters(220), Units.inchesToMeters(-29.5))

                ),

                new Pose2d(Units.inchesToMeters(304), Units.inchesToMeters(-29.5), new Rotation2d(0)),
                forwardConfig
        );
        return Trench2Ball;
    }

    public Trajectory getTrench3BallFarther() {
        Trajectory Trench3Ball;
        Trench3Ball = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(304), Units.inchesToMeters(-29.5), new Rotation2d(0)),
                List.of(
                        new Translation2d(Units.inchesToMeters(341), Units.inchesToMeters(-29.5))
                ),

                new Pose2d(Units.inchesToMeters(389), Units.inchesToMeters(-29.5), new Rotation2d(0)),
                forwardConfig
        );
        return Trench3Ball;
    }



    public Trajectory getToMediumFromPanel() {
        Trajectory ToMediumFromPanel;
        ToMediumFromPanel = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(316.694), Units.inchesToMeters(-28.691), new Rotation2d(Units.degreesToRadians(0))),
                List.of(
                        new Translation2d(Units.inchesToMeters(290.000), Units.inchesToMeters(-28.691))

                ),

                new Pose2d(Units.inchesToMeters(199.755), Units.inchesToMeters(-28.691), new Rotation2d(Units.degreesToRadians(0))),
                reverseFastConfig
        );
        return ToMediumFromPanel;

    }










}




//}

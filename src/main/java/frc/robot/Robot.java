package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.auton.Auton;
import frc.robot.configs.OM2025;
import frc.robot.operator.Operator;
import frc.robot.operator.Operator.OperatorConfig;
import frc.robot.pilot.Pilot;
import frc.robot.pilot.Pilot.PilotConfig;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.Claw.ClawConfig;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorConfig;
import frc.robot.subsystems.intake.IntakeDeploy;
import frc.robot.subsystems.intake.IntakeDeploy.IntakeDeployConfig;
import frc.robot.subsystems.intake.IntakeRoller;
import frc.robot.subsystems.intake.IntakeRoller.IntakeRollerConfig;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.subsystems.shoulder.Shoulder.ShoulderConfig;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Vision.VisionConfig;
import frc.spectrumLib.Rio;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.Telemetry.PrintPriority;
import frc.spectrumLib.util.CrashTracker;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;
import lombok.Getter;
import org.json.simple.parser.ParseException;

public class Robot extends TimedRobot {
    @Getter private static RobotSim robotSim;
    @Getter private static Config config;
    static Telemetry telemetry = new Telemetry();
    @Getter private static final Field2d field2d = new Field2d();

    public static class Config {
        public PilotConfig pilot = new PilotConfig();
        public OperatorConfig operator = new OperatorConfig();
        public SwerveConfig swerve = new SwerveConfig();
        public ElevatorConfig elevator = new ElevatorConfig();
        public ShoulderConfig shoulder = new ShoulderConfig();
        public ClawConfig claw = new ClawConfig();
        public IntakeRollerConfig intakeRoller = new IntakeRollerConfig();
        public IntakeDeployConfig intakeDeploy = new IntakeDeployConfig();
        public VisionConfig vision = new VisionConfig();
    }

    @Getter private static Pilot pilot;
    @Getter private static Operator operator;
    @Getter private static Auton auton;

    @Getter private static Swerve swerveSubsystem;
    @Getter private static Elevator elevatorSubsystem;
    @Getter private static Shoulder shoulderSubsystem;
    @Getter private static Claw clawSubsystem;
    @Getter private static IntakeDeploy intakeDeploySubsystem;
    @Getter private static IntakeRoller intakeRollerSubsystem;
    @Getter private static Vision visionSubsystem;

    @Getter private static Superstructure superstructure;

    public static boolean autonWarmedUp = false;

    public Robot() {
        super();
        Telemetry.start(true, true, PrintPriority.NORMAL);

        try {
            Telemetry.print("--- Robot Init Starting ---");
            robotSim = new RobotSim();

            /** Set up the config */
            switch (Rio.id) {
                default: // SIM and UNKNOWN
                    config = new OM2025();
                    break;
            }

            /**
             * Initialize the Subsystems of the robot. Subsystems are how we divide up the robot
             * code. Anything with an output that needs to be independently controlled is a
             * subsystem Something that don't have an output are also subsystems.
             */
            double canInitDelay = 0.1; // Delay between any mechanism with motor/can configs

            pilot = new Pilot(config.pilot);
            operator = new Operator(config.operator);

            swerveSubsystem = new Swerve(config.swerve);
            Timer.delay(canInitDelay);

            elevatorSubsystem = new Elevator(config.elevator);
            Timer.delay(canInitDelay);

            shoulderSubsystem = new Shoulder(config.shoulder);
            Timer.delay(canInitDelay);

            clawSubsystem = new Claw(config.claw);
            Timer.delay(canInitDelay);

            intakeDeploySubsystem = new IntakeDeploy(config.intakeDeploy);
            Timer.delay(canInitDelay);

            intakeRollerSubsystem = new IntakeRoller(config.intakeRoller);
            Timer.delay(canInitDelay);

            visionSubsystem = new Vision(config.vision);
            Timer.delay(canInitDelay);

            auton = new Auton();

            superstructure =
                    new Superstructure(
                            swerveSubsystem,
                            elevatorSubsystem,
                            shoulderSubsystem,
                            clawSubsystem,
                            intakeRollerSubsystem,
                            intakeDeploySubsystem);

            configureBindings();

            Telemetry.print("--- Robot Init Complete ---");

        } catch (Throwable t) {
            // intercept error and log it
            CrashTracker.logThrowableCrash(t);
            throw t;
        }

        RobotController.setBrownoutVoltage(Units.Volts.of(4.6));

        Telemetry.log("BuildConstants/ProjectName", BuildConstants.MAVEN_NAME);
        Telemetry.log("BuildConstants/BuildDate", BuildConstants.BUILD_DATE);
        Telemetry.log("BuildConstants/GitSHA", BuildConstants.GIT_SHA);
        Telemetry.log("BuildConstants/GitDate", BuildConstants.GIT_DATE);
        Telemetry.log("BuildConstants/GitBranch", BuildConstants.GIT_BRANCH);
        Telemetry.log(
                "BuildConstants/GitDirty",
                switch (BuildConstants.DIRTY) {
                    case 0 -> "All changes committed";
                    case 1 -> "Uncommitted changes";
                    default -> "Unknown";
                });
    }

    public void configureBindings() {
        pilot.LB
                .onTrue(
                        superstructure.configureButtonBinding(
                                Superstructure.WantedSuperState.CORAL_L4_LEFT_SCORE,
                                Superstructure.WantedSuperState.ALGAE_NET_PREP,
                                Superstructure.WantedSuperState.ALGAE_GROUND_INTAKE))
                .onFalse(
                        superstructure.setStateCommand(
                                Superstructure.WantedSuperState.DEFAULT_STATE));
        pilot.RB
                .onTrue(
                        superstructure.configureButtonBinding(
                                Superstructure.WantedSuperState.CORAL_L4_RIGHT_SCORE,
                                Superstructure.WantedSuperState.ALGAE_NET_SCORE,
                                Superstructure.WantedSuperState.CORAL_INTAKE_FLOOR))
                .onFalse(
                        superstructure.setStateCommand(
                                Superstructure.WantedSuperState.DEFAULT_STATE));
        pilot.LT
                .onTrue(
                        superstructure.configureButtonBinding(
                                Superstructure.WantedSuperState.CORAL_L3_LEFT_SCORE,
                                Superstructure.WantedSuperState.ALGAE_PROCESSOR_SCORE,
                                Superstructure.WantedSuperState.ALGAE_L3_INTAKE))
                .onFalse(
                        superstructure.setStateCommand(
                                Superstructure.WantedSuperState.DEFAULT_STATE));
        pilot.RT
                .onTrue(
                        superstructure.configureButtonBinding(
                                Superstructure.WantedSuperState.CORAL_L3_RIGHT_SCORE,
                                Superstructure.WantedSuperState.ALGAE_PROCESSOR_SCORE,
                                Superstructure.WantedSuperState.ALGAE_L3_INTAKE))
                .onFalse(
                        superstructure.setStateCommand(
                                Superstructure.WantedSuperState.DEFAULT_STATE));
        pilot.AButton.onTrue(
                        superstructure.configureButtonBinding(
                                Superstructure.WantedSuperState.CORAL_L2_LEFT_SCORE,
                                Superstructure.WantedSuperState.DEFAULT_STATE,
                                Superstructure.WantedSuperState.ALGAE_L2_INTAKE))
                .onFalse(
                        superstructure.setStateCommand(
                                Superstructure.WantedSuperState.DEFAULT_STATE));
        pilot.BButton.onTrue(
                        superstructure.configureButtonBinding(
                                Superstructure.WantedSuperState.CORAL_L2_RIGHT_SCORE,
                                Superstructure.WantedSuperState.DEFAULT_STATE,
                                Superstructure.WantedSuperState.ALGAE_L2_INTAKE))
                .onFalse(
                        superstructure.setStateCommand(
                                Superstructure.WantedSuperState.DEFAULT_STATE));

        pilot.dPadLeft.onTrue(clawSubsystem.forceSetHoldingCoralTrueCommand());
        pilot.dPadRight.onTrue(clawSubsystem.forceSetHoldingAlgaeTrueCommand());
        pilot.dPadUp.onTrue(intakeRollerSubsystem.forceSetHoldingCoralTrueCommand());
    }

    public void setupSmartDashboardData() {
        SmartDashboard.putData("Field2d", field2d);
    }

    @Override
    public void robotInit() {
        setupSmartDashboardData();
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    }

    /* ROBOT PERIODIC  */
    /**
     * This method is called periodically the entire time the robot is running. Periodic methods are
     * called every 20 ms (50 times per second) by default Since the robot software is always
     * looping you shouldn't pause the execution of the robot code This ensures that new values are
     * updated from the gamepads and sent to the motors
     */
    @Override
    public void robotPeriodic() {
        try {
            /**
             * Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled
             * commands, running already-scheduled commands, removing finished or interrupted
             * commands, and running subsystem periodic() methods. This must be called from the
             * robot's periodic block in order for anything in the Command-based framework to work.
             */
            CommandScheduler.getInstance().run();

            Telemetry.log("MatchTime", DriverStation.getMatchTime());
            field2d.setRobotPose(swerveSubsystem.getRobotPose());
        } catch (Throwable t) {
            // intercept error and log it
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        Telemetry.print("### Disabled Init Starting ### ");

        if (!autonWarmedUp) {
            Command autonStartCommand =
                    Commands.sequence(
                                    FollowPathCommand.warmupCommand(),
                                    PathfindingCommand.warmupCommand(),
                                    new InstantCommand(() -> Telemetry.log("Initialized?", true)))
                            .ignoringDisable(true);
            CommandScheduler.getInstance().schedule(autonStartCommand);
            autonWarmedUp = true;
        }

        Telemetry.print("### Disabled Init Complete ### ");
    }

    String autoName = "";

    @Override
    public void disabledPeriodic() {
        String newAutoName;
        boolean leftStart = true;
        List<PathPlannerPath> pathPlannerPaths = new ArrayList<>();
        newAutoName = auton.getAutonomousCommand().getName();
        leftStart = !newAutoName.endsWith(" - Right");

        if (newAutoName.equals("Do Nothing")) {
            field2d.getObject("Auto Routine").setPoses(new ArrayList<>());
            autoName = newAutoName;
            return;
        }

        // Remove " - Left" or " - Right" suffix if present
        if (newAutoName.endsWith(" - Left") || newAutoName.endsWith(" - Right")) {
            newAutoName = newAutoName.substring(0, newAutoName.lastIndexOf(" - "));
        }

        if (!autoName.equals(newAutoName)) {
            autoName = newAutoName;
            Telemetry.log("Auton Warmed Up", false);

            if (AutoBuilder.getAllAutoNames().contains(autoName)) {
                try {
                    pathPlannerPaths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
                } catch (IOException | ParseException e) {
                    Telemetry.print("Could not load path planner paths");
                }

                // Flip the paths if on red alliance
                Optional<Alliance> alliance = DriverStation.getAlliance();
                if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                    pathPlannerPaths =
                            pathPlannerPaths.stream()
                                    .map(PathPlannerPath::flipPath)
                                    .collect(Collectors.toList());
                }

                // Mirror the paths if starting on the right
                if (!leftStart) {
                    pathPlannerPaths =
                            pathPlannerPaths.stream()
                                    .map(PathPlannerPath::mirrorPath)
                                    .collect(Collectors.toList());
                }

                // Set the robot pose to the starting pose of the first path
                swerveSubsystem.resetPose(
                        pathPlannerPaths.get(0).getStartingHolonomicPose().orElse(new Pose2d()));

                // Warm up the starting path
                Command warmUpPath =
                        Commands.sequence(
                                        AutoBuilder.followPath(pathPlannerPaths.get(0))
                                                .withTimeout(0.5),
                                        Commands.runOnce(
                                                () -> {
                                                    Telemetry.print(
                                                            "Auton Warmed Up", PrintPriority.HIGH);
                                                    Telemetry.log("Auton Warmed Up", true);
                                                }))
                                .ignoringDisable(true);
                CommandScheduler.getInstance().schedule(warmUpPath);

                // Convert path points to poses
                List<Pose2d> poses = new ArrayList<>();
                for (PathPlannerPath path : pathPlannerPaths) {
                    poses.addAll(
                            path.getAllPathPoints().stream()
                                    .map(
                                            point ->
                                                    new Pose2d(
                                                            point.position.getX(),
                                                            point.position.getY(),
                                                            Rotation2d.kZero))
                                    .collect(Collectors.toList()));
                }
                field2d.getObject("Auto Routine").setPoses(poses);
            } else {
                field2d.getObject("Auto Routine").setPoses(new ArrayList<>());
            }
        }
    }

    @Override
    public void disabledExit() {
        Telemetry.print("### Disabled Exit### ");
    }

    /* AUTONOMOUS MODE (AUTO) */
    /**
     * This mode is run when the DriverStation Software is set to autonomous and enabled. In this
     * mode the robot is not able to read values from the gamepads
     */

    /** This method is called once when autonomous starts */
    @Override
    public void autonomousInit() {
        try {
            auton.init();
        } catch (Throwable t) {
            // intercept error and log it
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {
        auton.exit();
        Telemetry.print("@@@ Auton Exit @@@ ");
    }

    @Override
    public void teleopInit() {
        try {
            Telemetry.print("!!! Teleop Init Starting !!! ");
            field2d.getObject("path").setPoses(new ArrayList<>()); // clears auto visualizer

            Telemetry.print("!!! Teleop Init Complete !!! ");
            // climbPivot.startClimb();
        } catch (Throwable t) {
            // intercept error and log it
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {
        Telemetry.print("!!! Teleop Exit !!! ");
    }

    /* TEST MODE */
    /**
     * This mode is run when the DriverStation Software is set to test and enabled. In this mode the
     * is fully enabled and can move it's outputs and read values from the gamepads. This mode is
     * never enabled by the competition field It can be used to test specific features or modes of
     * the robot
     */

    /** This method is called once when test mode starts */
    @Override
    public void testInit() {
        try {

            Telemetry.print("~~~ Test Init Starting ~~~ ");

            Telemetry.print("~~~ Test Init Complete ~~~ ");
        } catch (Throwable t) {
            // intercept error and log it
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {
        Telemetry.print("~~~ Test Exit ~~~ ");
    }

    /* SIMULATION MODE */
    /**
     * This mode is run when the software is running in simulation and not on an actual robot. This
     * mode is never enabled by the competition field
     */

    /** This method is called once when a simulation starts */
    @Override
    public void simulationInit() {
        Telemetry.print("$$$ Simulation Init Starting $$$ ");

        Telemetry.print("$$$ Simulation Init Complete $$$ ");
    }

    /** This method is called periodically during simulation. */
    @Override
    public void simulationPeriodic() {}
}

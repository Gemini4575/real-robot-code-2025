// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Autos;
import frc.robot.Constants.FieldLocations;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.StartMotionSequence;
import frc.robot.commands.TelopSwerve;
import frc.robot.commands.Testing;
import frc.robot.commands.algea.EXO.OzDown;
import frc.robot.commands.algea.EXO.OzIntake;
import frc.robot.commands.algea.EXO.OzOutake;
import frc.robot.commands.algea.EXO.OzUp;
import frc.robot.commands.climbing.Climb;
import frc.robot.commands.coral.lili.AUTOCoral;
import frc.robot.commands.coral.lili.EXOCloseGateSlow;
import frc.robot.commands.coral.lili.LIPlaceCoral;
//import frc.robot.commands.coral.nora.INtakeFromHuman;
import frc.robot.commands.coral.nora.L1;
import frc.robot.commands.coral.nora.L2;
import frc.robot.commands.coral.nora.L3;
import frc.robot.commands.drive.AlineWheels;
import frc.robot.commands.drive.DriveTwoardsAprillTag;
import frc.robot.commands.drive.PathFindToPose;
import frc.robot.commands.drive.Stop;
import frc.robot.commands.drive.PathFindToPose.PathTarget;
// import frc.robot.commands.drive.TestTurnCommand;
import frc.robot.service.MotionService;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drive.DriveTrain;

import static frc.robot.datamodel.MotionDirective.apriltag;
import static frc.robot.datamodel.MotionDirective.drive;

// @Component
public class RobotContainer {

  Field2d visionPoseEstimate = new Field2d();
  private double up = 0.0;

  /* Controllers */
    private final Joystick driver = new Joystick(1);
    private final Joystick operator = new Joystick(2);
    private final Joystick testing = new Joystick(3);

  /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, JoystickConstants.BACK_BUTTON);

  /* Subsystems */
    private final NickClimbingSubsystem nc = new NickClimbingSubsystem();
    private final OzzyGrabberSubsystem g = new OzzyGrabberSubsystem();
    private final LiliCoralSubystem c = new LiliCoralSubystem();
    private final NoraArmSubsystem n = new NoraArmSubsystem();
    private final DriveTrain D = new DriveTrain();
    private final Vision V = new Vision();
    public final TestingSubsystem T = new TestingSubsystem(13);
    private final VisionSubsystem VS = new VisionSubsystem(V);

  /* Pathplanner stuff */
    private final SendableChooser<Command> PathplannerautoChoosers;
    private final SendableChooser<String> MyAutoChooser = new SendableChooser<>();
    private String MyAutoChooser_String;
    private final String DriveAndDrop1 = "Drive And Drop1";
    private final String Nothing = "Nothing";
    private final String DriveAndDRop2 = " Drive And Drop2";

  private final Field2d autoRobotPose = new Field2d();
  private final Field2d autoTargetPose = new Field2d();
  private final Field2d autoPath = new Field2d();

  private final MotionService motionService = new MotionService(D, c, VS);

  private OzUp ozGrabberUpCommand = new OzUp(g);

  public RobotContainer() {
    System.out.println("Starting RobotContainer()");
    NamedCommands.registerCommand("Drop Coral", new LIPlaceCoral(c));
    NamedCommands.registerCommand("Close Door", new EXOCloseGateSlow(c).withTimeout(2));
    NamedCommands.registerCommand("Is there Coral", new AUTOCoral(c));
    NamedCommands.registerCommand("Stop", new Stop(D));
    NamedCommands.registerCommand("Wheels", new AlineWheels(D));
    configureBindings();

    PathplannerautoChoosers = AutoBuilder.buildAutoChooser();

    MyAutoChooser.addOption("Drive And Drop1", DriveAndDrop1);
    MyAutoChooser.setDefaultOption("Nothing", Nothing);
    MyAutoChooser.addOption(" Drive And Drop2", DriveAndDRop2);
    SmartDashboard.putData(MyAutoChooser);

    configureLogging();

    SmartDashboard.putData("Auto Chooser", PathplannerautoChoosers);
    SmartDashboard.putData("Vision Pose Estimate", visionPoseEstimate);
    PathfindingCommand.warmupCommand().schedule();
    System.out.println("Ended RobotContainer()");
  }

  private void configureLogging() {
    SmartDashboard.putData("Auto Robot Pose", autoRobotPose);
    SmartDashboard.putData("Auto Target Pose", autoTargetPose);
    SmartDashboard.putData("Auto Path", autoPath);

    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      autoRobotPose.setRobotPose(pose);
    });

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      autoTargetPose.setRobotPose(pose);
    });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      // Do whatever you want with the poses here
      autoPath.getObject("Path").setPoses(poses);
    });
  }
  
  boolean teleFirst = false;

  public void teleopInit() {
    teleFirst = false;
    //new init(nc).schedule();
    D.setDefaultCommand(
        new TelopSwerve(
            D,
            () -> -driver.getRawAxis(Constants.JoystickConstants.LEFT_X_AXIS),
            () -> driver.getRawAxis(Constants.JoystickConstants.LEFT_Y_AXIS),
            () -> -driver.getTwist(),
            () -> operator.getRawButton(JoystickConstants.START_BUTTON)));
  }

  public void periodic() {

    SmartDashboard.putBoolean("Is flipped?", AutoBuilder.shouldFlip());

    if(driver.getRawButtonPressed(2)) {
      CommandScheduler.getInstance().cancelAll();
    }

    motionService.periodic();

    SmartDashboard.putNumber("Encoder", (nc.ClimbingMotor1.getEncoder().getPosition() + nc.ClimbingMotor2.getEncoder().getPosition()));

    if (!RobotState.isAutonomous()) {
      updateVisionEst();
    }
  }

  private void updateVisionEst() {
    var visionEst = V.getEstimatedGlobalPose();
    visionEst.ifPresent(
        est -> {
          // Change our trust in the measurement based on the tags we can see
          var estStdDevs = V.getEstimationStdDevs();

          D.addVisionMeasurement(
              est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);

          visionPoseEstimate.setRobotPose(est.estimatedPose.toPose2d());
        });
  }
  double autoFirst = 0.0;
  public void autonomousInit() {
    MyAutoChooser_String = MyAutoChooser.getSelected();
    autoFirst = 0.0;
  }

  public void autonomousPeriodic() {
    if (g.isHangingLoose() && !ozGrabberUpCommand.isScheduled() && ozGrabberUpCommand.isFinished()) {
      System.out.println("Grabber is loose, fixing..");
      //ozGrabberUpCommand.schedule();
    }
    if(autoFirst == 0) {
      switch (MyAutoChooser_String) {
        case DriveAndDrop1:
          new  
          StartMotionSequence(motionService, Autos.AUTO_CORAL1).schedule();
        break;

        case Nothing:

        break;

        case DriveAndDRop2:
          new StartMotionSequence(motionService, Autos.AUTO_CORAL2).schedule();
        break;
        default:
          new  
          StartMotionSequence(motionService, Autos.AUTO_CORAL1).schedule();
        break;
      }
      autoFirst++;
    }
  }

  private void configureBindings() {
    System.out.println("Starting configureBindings()");

    /* Driver Controls */
      zeroGyro.onTrue(new InstantCommand(() -> D.ResetDrives()));
    /* Operator Controls */
    //  new JoystickButton(operator, JoystickConstants.GREEN_BUTTON)
    //    .onTrue(new DriveTwoardsAprillTag(vision, s_swerve));

      new JoystickButton(operator, JoystickConstants.BLUE_BUTTON)
        .onTrue(new LIPlaceCoral(c));
        // .and(g.BeamBreak())
        // .onTrue(new Proceser(g, new JoystickButton(operator, JoystickConstants.BLUE_BUTTON)))
        // .or(new JoystickButton(operator, JoystickConstants.BLUE_BUTTON))
        // .and(g.FalseBeamnBreak())
        // .onTrue(new IntakeAlgae(g));

      // new JoystickButton(operator, 1).//JoystickConstants.GREEN_BUTTON).
      //   and(c.Coral()).
      //   onTrue(new LIPlaceCoral(c, s_swerve));

      new JoystickButton(operator, JoystickConstants.RIGHT_BUMPER).onTrue(new Testing(T));

      new JoystickButton(operator, JoystickConstants.YELLOW_BUTTON)
        .onTrue(new OzDown(g));
      new JoystickButton(driver, 9)
        .onTrue(ozGrabberUpCommand); 
      new JoystickButton(driver, 10)
        .onTrue(new OzIntake(g));
        new JoystickButton(driver, 11)
        .onTrue(new OzOutake(g));


      new JoystickButton(operator, JoystickConstants.RED_BUTTON)
        .onTrue(new Climb(nc));
      
      new JoystickButton(operator, JoystickConstants.GREEN_BUTTON)
          .onTrue(new PathFindToPose(D, PathTarget.ALGAE_INTAKE));

      //new JoystickButton(operator, JoystickConstants.GREEN_BUTTON)
      //  .onTrue(new StartMotionSequence(motionService, Autos.AUTO_WITH_CAM)/*new INtakeFromHuman(n, visionSubsystem)*/);

      //new JoystickButton(operator, JoystickConstants.BACK_BUTTON)
      //  .onTrue(new DriveTwoardsAprillTag(V, D));
      //new JoystickButton(operator, JoystickConstants.BACK_BUTTON).onTrue(new Turn(s_swerve));

      // new JoystickButton(operator, JoystickConstants.GREEN_BUTTON)
      //   .onTrue(new 
      //     StartMotionSequence(motionService, 
      //     drive(1), turn(90), drive(1), turn(90), 
      //     drive(1), turn(90), drive(1), turn(90)));
      
      //
          // new JoystickButton(driver, 10)
          // .onTrue(new 
          //   StartMotionSequence(motionService, turn(90)));

      // new JoystickButton(driver, 11)
      //   .onTrue(new 
      //     StartMotionSequence(motionService, turn(-90))); 

          new JoystickButton(driver, 12)
          .onTrue(new 
            StartMotionSequence(motionService, drive(1)));

      new JoystickButton(operator, JoystickConstants.POV_DOWN)
        .onTrue(new StartMotionSequence(motionService, apriltag()));

      // new JoystickButton(operator, JoystickConstants.POV_LEFT)
      //   .onTrue(new StartMotionSequence(motionService, new MotionDirective(MotionType.CAMERA_STRAFE)));

      // new JoystickButton(operator, JoystickConstants.POV_RIGHT)
      //   .onTrue(new StartMotionSequence(motionService, new MotionDirective(MotionType.CAMERA_ROTATE)));

      // new JoystickButton(operator, JoystickConstants.POV_UP)
      //   .onTrue(new StartMotionSequence(motionService, new MotionDirective(MotionType.CAMERA_ALL)));

    // Supplier<Pose2d> bestTargetSupplier = () -> {
    //   var target = vision.getTargets();
    //   if (target != null && kTagLayout.getTagPose(target.fiduc                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           ialId).isPresent()) {
    //     SmartDashboard.putString("Targeting tag", String.valueOf(target.getFiducialId()));
    //     return kTagLayout.getTagPose(target.fiducialId).get().toPose2d();
    //   }
    //   return null;
    // };
    
    // alternative option using PathPlanner - only if target is far enough
    // new JoystickButton(operator, JoystickConstants.BLUE_BUTTON)
    // // //.and(() -> {
    // // // return
    // bestTargetSupplier.get().getTranslation().getDistance(s_swerve.getPose().getTranslation())
    // > 2.0;
    // // //})
    // .onTrue(new PathFindToPose(s_swerve, bestTargetSupplier));

    System.out.println("Ended configureBindings()");
  }

  public void teleopPeriodic() { 
    c.JoyControll(operator.getRawAxis(JoystickConstants.LEFT_Y_AXIS));
    nc.JoyClimb1(testing.getRawAxis(JoystickConstants.RIGHT_Y_AXIS), testing.getRawButton(JoystickConstants.START_BUTTON));
    nc.JoyClimb2(testing.getRawAxis(JoystickConstants.LEFT_Y_AXIS), operator.getRawButton(JoystickConstants.BACK_BUTTON));    
  
  if(operator.getRawButtonPressed(JoystickConstants.POV_UP)){
      up++;
      teleFirst = true;
    }
    if(up == 1) {
      if(teleFirst) {
        up = 100;
        teleFirst = false;
        new L1(n).schedule();
      }
    } else if (up == 2) {
      if(teleFirst) {
        up = 100;
        teleFirst = false;
        new L2(n).schedule();
      }
    } else if (up == 3) {
      if(teleFirst) {
        up = 100;
        teleFirst = false;
        new L3(n).schedule();
      }
    } else if (up > 3) {
      up = 0;
    }
  }

  public Command getAutonomousCommand() {
    return PathplannerautoChoosers.getSelected();
  }

  public void onDisable() {
    motionService.stop(true);
  }
}

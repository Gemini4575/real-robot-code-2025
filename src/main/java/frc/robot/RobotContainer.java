// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Autos;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.TelopSwerve;
import frc.robot.commands.Testing;
import frc.robot.commands.algea.EXO.Down;
import frc.robot.commands.climbing.Climb;
import frc.robot.commands.coral.lili.AUTOCoral;
import frc.robot.commands.coral.lili.AUTOCoralFalse;
import frc.robot.commands.coral.lili.EXOCloseGateSlow;
import frc.robot.commands.coral.lili.LIPlaceCoral;
import frc.robot.commands.coral.lili.LiAutoPlaceCoral;
import frc.robot.commands.drive.AlineWheels;
import frc.robot.commands.drive.AlineWheels2;
import frc.robot.commands.drive.DriveTwoardsAprillTag;
import frc.robot.commands.drive.PathFindToPose;
import frc.robot.commands.drive.PatrolCoralStations;
import frc.robot.commands.drive.Stop;
import frc.robot.commands.drive.PathFindToPose.PathTarget;
// import frc.robot.commands.drive.TestTurnCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drive.DriveTrain;

import static frc.robot.datamodel.MotionDirective.apriltag;
import static frc.robot.datamodel.MotionDirective.drive;

import static frc.robot.Constants.JoystickConstants.*;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.EstimatedRobotPose;

// @Component
public class RobotContainer {

  Field2d visionPoseEstimate = new Field2d();

  /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);
    private final Joystick climber = new Joystick(2);
    private final Joystick testing = new Joystick(3);

  /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, JoystickConstants.BACK_BUTTON);

  /* Subsystems */
    private final NickClimbingSubsystem nc = new NickClimbingSubsystem();
    private final OzzyGrabberSubsystem g = new OzzyGrabberSubsystem();
    private final LiliCoralSubystem c = new LiliCoralSubystem();
    private final DriveTrain D = new DriveTrain();
    private final Vision V = new Vision();
    private final VisionSubsystem VS = new VisionSubsystem(V);

  /* Pathplanner stuff */
    private final SendableChooser<Command> PathplannerautoChoosers;
    private final SendableChooser<String> MyAutoChooser = new SendableChooser<>();

  private final Field2d autoRobotPose = new Field2d();
  private final Field2d autoTargetPose = new Field2d();
  private final Field2d autoPath = new Field2d();


  // private OzUp ozGrabberUpCommand = new OzUp(g);

  public RobotContainer() {
    System.out.println("Starting RobotContainer()");
    NamedCommands.registerCommand("Drop Coral", new LiAutoPlaceCoral(c));
    NamedCommands.registerCommand("Drop and Close Coral", new LIPlaceCoral(c));
    NamedCommands.registerCommand("Close Door", new EXOCloseGateSlow(c));
    NamedCommands.registerCommand("Is there Coral", new AUTOCoral(c));
    NamedCommands.registerCommand("Is there not Coral", new AUTOCoralFalse(c));
    NamedCommands.registerCommand("Stop", new Stop(D));
    NamedCommands.registerCommand("Wheels", new AlineWheels(D));
    NamedCommands.registerCommand("Wheels long", new AlineWheels2(D));

    new EventTrigger("Drop Coral").onTrue(new LiAutoPlaceCoral(c));
    


    configureBindings();

    PathplannerautoChoosers = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("[Robot]Auto Chosers", PathplannerautoChoosers);
    SmartDashboard.putString("[Robot]Note", "");


    configureLogging();

    SmartDashboard.putData("[Robot]Vision Pose Estimate", visionPoseEstimate);
    PathfindingCommand.warmupCommand().schedule();
    System.out.println("Ended RobotContainer()");
  }

  private void configureLogging() {
    SmartDashboard.putData("[Robot]Auto Robot Pose", autoRobotPose);
    SmartDashboard.putData("[Robot]Auto Target Pose", autoTargetPose);
    SmartDashboard.putData("[Robot]Auto Path", autoPath);

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
    // new init(nc).schedule();
    D.setDefaultCommand(
        new TelopSwerve(
            D,
            () -> -driver.getRawAxis(Constants.JoystickConstants.LEFT_X_AXIS),
            () -> driver.getRawAxis(Constants.JoystickConstants.LEFT_Y_AXIS),
            () -> -driver.getTwist(),
            () -> operator.getRawButton(JoystickConstants.START_BUTTON)));
  }
  @AutoLogOutput
  String noteString;
  public void periodic() {
    noteString = SmartDashboard.getString("[Robot]Note", "");

    SmartDashboard.putBoolean("[Robot]Is flipped?", AutoBuilder.shouldFlip());

    if(driver.getRawButtonPressed(2)) {
      CommandScheduler.getInstance().cancelAll();
    }



    //if (!RobotState.isAutonomous()) {
      updateVisionEst();
   // }
  }

  private void updateVisionEst() {
    var visionEst = V.getEstimatedGlobalPose();
    updateLocationWithVision(visionEst);
    var visionEstColor = V.getEstimatedGlobalPoseColor();
    updateLocationWithVision(visionEstColor);
  }

  private void updateLocationWithVision(Optional<EstimatedRobotPose> visionEst) {
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
    autoFirst = 0.0;
  }

  public void autonomousPeriodic() {
    // if (g.isHangingLoose() && !ozGrabberUpCommand.isScheduled() && ozGrabberUpCommand.isFinished()) {
    //   System.out.println("Grabber is loose, fixing..");
    //   //ozGrabberUpCommand.schedule();
    // }
  }

  private void configureBindings() {
    System.out.println("Starting configureBindings()");

    /* Driver Controls */
      zeroGyro.onTrue(new InstantCommand(() -> D.ResetDrives()));
    /* Operator Controls */
      new JoystickButton(operator, JoystickConstants.BLUE_BUTTON)
        .onTrue(new LIPlaceCoral(c));

      

      // new JoystickButton(operator, JoystickConstants.YELLOW_BUTTON)
      //   .onTrue(new OzDown(g));
      // new JoystickButton(driver, 9)
      //   .onTrue(ozGrabberUpCommand); 
      // new JoystickButton(driver, 10)
      //   .onTrue(new OzIntake(g));
      //   new JoystickButton(driver, 11)
      //   .onTrue(new OzOutake(g));


      // new JoystickButton(operator, JoystickConstants.RED_BUTTON)
      //   .onTrue(new Climb(nc));
      
      new JoystickButton(operator, JoystickConstants.GREEN_BUTTON)
          .whileTrue(new PathFindToPose(D, PathTarget.LEFT_HUMAN_STATION));
      // new JoystickButton(driver, 12).whileTrue(new PatrolCoralStations(D));

      //new JoystickButton(operator, JoystickConstants.GREEN_BUTTON)
      //  .onTrue(new StartMotionSequence(motionService, Autos.AUTO_WITH_CAM)/*new INtakeFromHuman(n, visionSubsystem)*/);

      new JoystickButton(testing, JoystickConstants.BACK_BUTTON)
        .onTrue(new DriveTwoardsAprillTag(V, D));

      new JoystickButton(testing, JoystickConstants.YELLOW_BUTTON)
        .onTrue(new LiAutoPlaceCoral(c));

      new JoystickButton(testing, JoystickConstants.GREEN_BUTTON)
        .onTrue(new EXOCloseGateSlow(c));

      new JoystickButton(operator, RED_BUTTON)
        .onTrue(new Down(g));
      
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

          

    // Supplier<Pose2d> bestTargetSupplier = () -> {
    //   var target = vision.getTargets();
    //   if (target != null && kTagLayout.getTagPose(target.fiduc                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           ialId).isPresent()) {
    //     SmartDashboard.putString("[Robot]Targeting tag", String.valueOf(target.getFiducialId()));
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
    // c.JoyControll(operator.getRawAxis(JoystickConstants.LEFT_Y_AXIS));
    g.joy(MathUtil.applyDeadband(operator.getRawAxis(JoystickConstants.LEFT_Y_AXIS), 0.5) * 0.5);
    // g.joy1(MathUtil.applyDeadband(climber.getRawAxis(JoystickConstants.LEFT_Y_AXIS), 0.2));
    nc.JoyClimb1(MathUtil.applyDeadband(climber.getRawAxis(JoystickConstants.RIGHT_Y_AXIS), 0.5), climber.getRawButton(JoystickConstants.START_BUTTON));
    nc.JoyClimb2(MathUtil.applyDeadband(climber.getRawAxis(JoystickConstants.LEFT_Y_AXIS), 0.5), climber.getRawButton(JoystickConstants.BACK_BUTTON));    
    
    if(operator.getRawButton(JoystickConstants.LEFT_BUMPER)) {
      g.intake();
    } else if (operator.getRawButton(JoystickConstants.RIGHT_BUMPER)) {
      g.outake();
    } else {
      g.stop();
    }
  }

  public Command getAutonomousCommand() {
    return PathplannerautoChoosers.getSelected();
  }

  public void onDisable() {}
}

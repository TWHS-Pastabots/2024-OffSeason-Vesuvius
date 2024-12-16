package frc.robot;

import java.util.function.BiPredicate;

import org.littletonrobotics.junction.LoggedRobot;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
 
import com.fasterxml.jackson.databind.jsontype.impl.ClassNameIdResolver;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import frc.robot.Ports;
import frc.robot.commands.ElevatorBotCommand;
import frc.robot.commands.HawkTuah;
import frc.robot.commands.PickUpCommand;
import frc.robot.commands.RotationCommand;
import frc.robot.commands.ScoreCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.IO.LED;

import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.elevator.Elevator.ElevatorState;
import frc.robot.subsystems.swerve.Drivebase;
import frc.robot.subsystems.swerve.Drivebase.DriveState;
import frc.robot.subsystems.vision.CameraSystem;

public class Robot extends LoggedRobot {
  //all the initialing for the systems
  private Drivebase drivebase;
  private Elevator elevator;
  // private LED litty;
  private CameraSystem camSystem;
  private Claw claw;
  private Climber climber;
  private static GenericHID board;
  private static Joystick stick;
  private static XboxController driver;
  private static XboxController operator;
  private PneumaticHub junoHub = new PneumaticHub(20);
  private final Compressor m_compressor = new Compressor(PneumaticsModuleType.REVPH);
  //initialization of the auton chooser in the dashboard
  private Command m_autoSelected;

  private HawkTuah hawkTuah;
  private ScoreCommand scoreCommand;
  private PickUpCommand pickUpCommand;
  private RotationCommand rotationCommand;
  private ElevatorBotCommand elevatorBotCommand;

  private int tagID;


  // Double targetRange = null;
  // Double targetAngle = null;

   
  //that is a chooser for the autons utilizing the sendableChooser which allows us to choose the auton commands
  private SendableChooser<Command> m_chooser = new SendableChooser<>();


  @Override
  public void robotInit() {

    tagID = 1;

    drivebase = Drivebase.getInstance();
    elevator = Elevator.getInstance();
    claw = Claw.getInstance();
    climber = Climber.getInstance();
    camSystem = CameraSystem.getInstance();

    scoreCommand = new ScoreCommand();
    pickUpCommand = new PickUpCommand();
    rotationCommand = new RotationCommand(Math.toRadians(90));
    elevatorBotCommand = new ElevatorBotCommand();
    hawkTuah =  new HawkTuah();

    NamedCommands.registerCommand("scoreCommand", scoreCommand);
    NamedCommands.registerCommand("pickUpCommand", pickUpCommand);
    NamedCommands.registerCommand("rotationCommand", rotationCommand);
    NamedCommands.registerCommand("elevatorBotCommand", elevatorBotCommand);
    NamedCommands.registerCommand("outtake", hawkTuah);
    // litty = LED.getInstance();
    camSystem = CameraSystem.getInstance();
    camSystem.AddCamera(new PhotonCamera("FrontCam"), new Transform3d(
        new Translation3d(0.11, 0.0, -.50), new Rotation3d(0.0, Math.toRadians(15), 0.0))
        ,  true);

    camSystem.AddCamera(new PhotonCamera("BackCam"),  new Transform3d(
      new Translation3d(-0.31, 0.0, -1.051), new Rotation3d(0.0, 0.0, 0.0)),
       true);

    board = new GenericHID(2);
    stick = new Joystick(2);

    driver = new XboxController(0);
    operator = new XboxController(1);
   
    m_chooser.addOption("RPS", new PathPlannerAuto("RPS"));
    m_chooser.addOption("FieldTest", new PathPlannerAuto("FieldTest"));
    m_chooser.addOption("TestPathX", new PathPlannerAuto("TestPathX"));
    m_chooser.addOption("RedScore1", new PathPlannerAuto("OneRedScore"));
    m_chooser.addOption("Green 3", new PathPlannerAuto("Green 3"));
    m_chooser.addOption("GreenTest", new PathPlannerAuto("GreenTest"));

    SmartDashboard.putData("Auto choices", m_chooser);


  }
  
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("X-coordinate", drivebase.getPose().getX());
    SmartDashboard.putNumber("Y-coordinate", drivebase.getPose().getY());
    SmartDashboard.putNumber("Elevator Position", elevator.getPosition(elevator.encoderL));
    SmartDashboard.putNumber("Elevator Position R", elevator.getPosition(elevator.encoderR));
    SmartDashboard.putNumber("Absolute Encoder ", elevator.encoderA.getPosition());

      // Pose2d cameraPositionTele = camSystem.calculateRobotPosition();

      //  Pose2d posTele = drivebase.updateOdometry(cameraPositionTele);

      // junoHub.enableCompressorDigital();
      // m_compressor.enableDigital();
      //   SmartDashboard.putNumber("Odometry X", posTele.getX());
      //   SmartDashboard.putNumber("Odometry Y", posTele.getY());

      //this is getting the data from the cameras through the cameraSystem class 
    //  if (camSystem.getCamera(0).isConnected()) {
    //         PhotonPipelineResult backResult = camSystem.getResult(0);      
    //         if (backResult.hasTargets()) {
    //             PhotonTrackedTarget target = backResult.getBestTarget();
    //             Transform3d bestCameraToTarget = target.getBestCameraToTarget();
    //             double distance  = bestCameraToTarget.getTranslation().getNorm();
    //             SmartDashboard.putString("Back Camera Target", "Yes Targets");
    //             SmartDashboard.putNumber("Back to Target", distance);
    //             SmartDashboard.putNumber("Back Camera Target Yaw", target.getYaw());
    //             SmartDashboard.putNumber("Back Camera Target Pitch", target.getPitch());
    //             SmartDashboard.putNumber("Back Camera Target Area", target.getArea());
    //             SmartDashboard.putNumber("ID", target.getFiducialId());

    //         } else if(backResult.hasTargets() == false) {
    //             SmartDashboard.putString("Back Camera Target", "No Targets");
    //         }
    //     } 
      //testing the valuies that the camera gives us and outputing it into the dashboard
      // Pose2d cameraPosition = camSystem.calculateRobotPosition();
      // SmartDashboard.putNumber("Camera X Position", cameraPosition.getX());
      // SmartDashboard.putNumber("Camera Y Position", cameraPosition.getY());
      // SmartDashboard.putNumber("Camera Heading", cameraPosition.getRotation().getDegrees());


      // SmartDashboard.putNumber("Analog", VCompressor.getAnalogVoltage());
      // SmartDashboard.putNumber("Compressor Pressure", VCompressor.getPressure());
      // SmartDashboard.putBoolean("Compressor", VCompressor.isEnabled());
      // SmartDashboard.putNumber("Compressor Current", VCompressor.getCurrent());
    
    CommandScheduler.getInstance().run();
    drivebase.periodic();
    camSystem.ChangeCamOffset(elevator.getPosition(elevator.encoderR));
    //putting all of the info from the subsystems into the dashvoard so we can test things
    SmartDashboard.putNumber("Gyro Angle:", (drivebase.getHeading() + 90) % 360);
    // SmartDashboard.putNumber("CRF", claw.cylinderR.getFwdChannel());
    // SmartDashboard.putNumber("CRR", claw.cylinderR.getRevChannel());
    // SmartDashboard.putNumber("CLF", claw.cylinderL.getFwdChannel());
    // SmartDashboard.putNumber("CLR", claw.cylinderL.getRevChannel());
    // SmartDashboard.putNumber("Compressor C", claw.isCompressorEnabled());
    // SmartDashboard.putBoolean("Compressor Enabled?", claw.isCompressorEnabled());
    // SmartDashboard.putNumber("Compressor V", claw.compressor.getAnalogVoltage());
    // SmartDashboard.putNumber("Compressor C", claw.compressor.getCurrent());
    //  SmartDashboard.putNumber("Compressor P", claw.compressor.getPressure());

    // SmartDashboard.putNumber("X-coordinate", drivebase.getPose().getX());
    // SmartDashboard.putNumber("Y-coordinate", drivebase.getPose().getY());

 
  }

  @Override
  public void autonomousInit() {
    //getting the value we chose from the dashboard and putting it into motion in the auton
    m_autoSelected = m_chooser.getSelected();

    // drivebase.resetOdometry(PathPlannerAuto.getStaringPoseFromAutoFile(m_chooser.getSelected().getName()));
    
    
//schedules the command so it actually begins moving
    if (m_autoSelected != null) {
      m_autoSelected.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    //updating the intake for the autointake command
    //using cameras to calculate the robot position instead of odometry.
    //we use a mix of odometry + camera positions to calculate the robot position
    
    // Pose2d cameraPosition = camSystem.calculateRobotPosition(); 
    // Pose2d pose = drivebase.updateOdometry(cameraPosition);
    camSystem.AddVisionMeasurements(Drivebase.poseEstimator);


    SmartDashboard.putNumber("Auto X", drivebase.getPose().getX());
    SmartDashboard.putNumber("Auto Y", drivebase.getPose().getY());
    // SmartDashboard.putNumber("Odometry X", pose.getX());
    // SmartDashboard.putNumber("Odometry Y", pose.getY());
  }

  @Override
  public void teleopInit() {
    //as soon as we begin teleop we desable the auton selection
    // litty.setBlue();
    if (m_autoSelected != null) {
      m_autoSelected.cancel();
    }

    // Translation2d testxy = new Translation2d(16.57 - 14.7, 5.54);
    // Rotation2d testRot = new Rotation2d(0);
    // Pose2d test = new Pose2d(testxy, testRot);
    // drivebase.resetOdometry(test);
  }

  @Override
  public void teleopPeriodic() {
    // elevator.updatePose();
    /* DRIVE CONTROLS */

    // SmartDashboard.putNumber("Elevator Position", elevator.getPosition(elevator.encoderL));
    // SmartDashboard.putNumber("Elevator Position R", elevator.getPosition(elevator.encoderR));
    //setting inputs for driving through the driver controller
    double ySpeed = drivebase.inputDeadband(-driver.getLeftX());
    double xSpeed = drivebase.inputDeadband(driver.getLeftY());
    double rot = drivebase.inputDeadband(-driver.getRightX());

  //   double ySpeed = drivebase.inputDeadband(-stick.getX() * .17);
  //   double xSpeed = drivebase.inputDeadband(stick.getY() * .17);
  //   double rot = drivebase.inputDeadband(stick.getRawAxis(3)*.17);

  //   if(stick.getZ() > 0.0){
  //   rot = drivebase.inputDeadband(stick.getZ()*.17);
  //  }else{
  //   rot = drivebase.inputDeadband(-stick.getRawAxis(3)*.17);
  //  }
    
   


    // if(driver.getRightTriggerAxis()>0.0){
    //   elevator.elevatorOn();
      
    // }else if(driver.getLeftTriggerAxis()>0.0){
    //   elevator.elevatorReverse();
    // }else{
    //   elevator.elevatorOff();
    // }
    

    //  if(driver.getAButton()){
    //   elevator.setElevatorState(ElevatorState.BOT);
    //   elevator.updatePose();
    //  }
    if(driver.getBButton())
    {
      tagID = 1;
    }
    else if(driver.getYButton())
    {
      tagID = 2;
    }
    else if(driver.getXButton())
    {
      tagID = 3;
    }

    if (driver.getLeftTriggerAxis() > .1)
    {
      Double yaw = camSystem.getYawForTag(0, tagID);
      if(yaw != null)
      {
        rot = -yaw * .02 * Constants.DriveConstants.kMaxAngularSpeed;
      }
    }  


     if(driver.getRightBumper()){
      claw.setWheelsOn();
    }else if(driver.getLeftBumper()){
      claw.setWheelsReverse();
    }else{
      claw.setWheelsOff();
    }


    drivebase.drive(xSpeed, ySpeed, rot);
    //  if(driver.getXButton()){
    //   climber.setTommysWheelsOn();
    // }else if(driver.getBButton()){
    //   climber.setTommysWheelsReverse();
    // }else{
    //   climber.setTommysWheelsOff();
    // }

      





    //using buttons to rotate the robot by increments of 90 degrees
    // if (driver.getAButton()) {
    //   drivebase.currHeading = -1;
    //   drivebase.rotateTo(xSpeed, ySpeed, 180);
    // } else if (driver.getBButton()) {
    //   drivebase.currHeading = -1;
    //   drivebase.rotateTo(xSpeed, ySpeed, 270);
    // } else if (driver.getYButton()) {
    //   drivebase.currHeading = -1;
    //   drivebase.rotateTo(xSpeed, ySpeed, 0);
    // } else if (driver.getXButton()) {
    //   drivebase.currHeading = -1;
    //   drivebase.rotateTo(xSpeed, ySpeed, 90);
    // } else if (driver.getLeftTriggerAxis() > 0) {
    // } else {
    //   drivebase.currHeading = -1;
    //   drivebase.drive(xSpeed, ySpeed, rot);
    // }


  
    if (driver.getPOV() == 0) {
      drivebase.zeroHeading();
    }
    // if (driver.getAButton()){
    //   Double yaw = camSystem.getYawForTag(0, 2);

    // }


    // if (driver.getRightTriggerAxis() > 0) {
    //   drivebase.setDriveState(DriveState.SLOW);
    // } 
    // //getting yaw from the tag to rotate towards it. The robot will allign itself with the 
    // if(driver.getLeftTriggerAxis() > 0)
    // {
    //   Double yaw = camSystem.getYawForTag(1, 4);
    //   targetRange = camSystem.getTargetRange(1, 4);
    //   if(yaw !=null)
    //   {
    //     rot =  -yaw * .002 * Constants.DriveConstants.kMaxAngularSpeed;
    //   }
    //   // if(targetRange != null){
    //   //   xSpeed = (targetRange - 2.5) * .1 * Constants.DriveConstants.kMaxSpeedMetersPerSecond;
    //   // }
      
      
    // }
    
    // if(targetRange != null)
    // {
    //   SmartDashboard.putNumber("TargetRange", targetRange);
    // }
    // if(targetAngle != null)
    // {
    //   SmartDashboard.putNumber("Target Angle", targetAngle);
    // }
    // if(camSystem.getResult(1).hasTargets() && camSystem.getResult(1).getBestTarget() != null){
    //   SmartDashboard.putNumber("TargetPitch", Units.degreesToRadians(camSystem.getResult(1).getBestTarget().getPitch()));
    // }
    // if(driver.getRightBumper()){
    //   claw.setWheelsOn();
    // }else if (driver.getLeftBumper()){
    //   claw.setWheelsReverse();
    // }else{
    //   claw.setWheelsOff();
    // }

    // if(operator.getRightTriggerAxis()> 0.0){
    //   elevator.elevatorOn();
    // }else if(operator.getLeftTriggerAxis()>0.0){
    //   elevator.elevatorReverse();
    // }else{
    //   elevator.elevatorOff();
    // }

    // if(board.getRawButton(1)){
    //   elevator.elevatorOn();
    //   SmartDashboard.putString("Button 1 pressed", "Y");
    // }else{
    //   SmartDashboard.putString("Button 1 pressed", "N");
    // }
    // if(board.getRawButton(2)){
    //   elevator.elevatorReverse();
    //   SmartDashboard.putString("Button 2 pressed", "Y");
    // }else{
    //   SmartDashboard.putString("Button 2 pressed", "N");
    // }
    // if(board.getRawButton(3)){
    //   claw.setWheelsOn();
    //   SmartDashboard.putString("Button 3 pressed", "Y");
    // }else{
    //   SmartDashboard.putString("Button 3 pressed", "N");
    // }
    

    // if(jun.getRawButton(5)){
    //   claw.setWheelsOff();
    //   elevator.elevatorOff();
    // }
    if (driver.getPOV() == 90){
      elevator.setElevatorState(ElevatorState.TOP);
      elevator.updatePose();
    } else if(driver.getAButton()){
      elevator.setElevatorState(ElevatorState.MID);
            elevator.updatePose();

    } else if(driver.getPOV() == 270){
      elevator.setElevatorState(ElevatorState.BOT);
            elevator.updatePose();

    }else if(driver.getPOV() == 180){
      elevator.setElevatorState(ElevatorState.GROUND);
      elevator.updatePose();
    }

    

    


  }
    


  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }

}
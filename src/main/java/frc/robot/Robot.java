/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Importer les librairies
package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;

//Définir les composantes
public class Robot extends TimedRobot {
  private PIDController m_pidController;
  private PIDController m_pidController2;
  private WPI_VictorSPX m_leftMotor;
  private WPI_VictorSPX m_rightMotor;
  private DifferentialDrive m_robotDrive;
  private Joystick m_stick;
  
  private AHRS ahrs; 

  private static final double kP = -.075;
  private static final double kP2 = -.075;
  private static final double kI = -0.00;
  private static final double kD = -0.0;
  private static final double kD2 = -0.0;





//Accéder aux données du limelight
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");


//Commendes a éxécuter seulement au démarrage
  @Override
  public void robotInit() {
    m_leftMotor = new WPI_VictorSPX(6);
    m_rightMotor = new WPI_VictorSPX(9);
    m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
    m_stick = new Joystick(0);
    ahrs = new AHRS(SPI.Port.kMXP);
    m_pidController = new PIDController(kP, kI, kD, 0.02);
    m_pidController2 = new PIDController(kP2, kI, kD2, 0.02);
    m_pidController.setSetpoint(90.0);
m_pidController2.setSetpoint(0.0);
  }


//Conduire avec 'arcade drive'
  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(m_stick.getY(), -m_stick.getX());


//Lire les données du limelight
double x = tx.getDouble(0.0);
double y = ty.getDouble(0.0);
double area = ta.getDouble(0.0);


//Lire les données du navX
double anglemesure = ahrs.getYaw();
double vitesseangulaire = ahrs.getRawGyroX();


//Calculer les vitesses (Lignes 110 à 117)
double pidOut2 = m_pidController2.calculate(anglemesure);
double pidOut = m_pidController.calculate(anglemesure);


//Poster au Smart Dashboard les donées du navX
SmartDashboard.putNumber("anglemesure", anglemesure);
SmartDashboard.putNumber("vitesseangulaire", vitesseangulaire);
SmartDashboard.putNumber("pidOut2", pidOut2);
SmartDashboard.putNumber("pidOut", pidOut);


//Poster au Smart Dashboard les données du limelight
SmartDashboard.putNumber("LimelightX", x);
SmartDashboard.putNumber("LimelightY", y);
SmartDashboard.putNumber("LimelightArea", area);


//Système de controle automatique
if (m_stick.getRawButton(1)) {
    m_robotDrive.arcadeDrive(-0.7, -x*0.04);
  }


//Tourner a un angle
if (m_stick.getRawButton(2)) {
  ahrs.reset();
//   m_pidController.setSetpoint(90.0);
// m_pidController2.setSetpoint(0.0);

} else {
  
}


if (m_stick.getRawButton(3)) {
  //double erreur = 90.0 - anglemesure;
  m_robotDrive.arcadeDrive(0.0, pidOut);
}


if (m_stick.getRawButton(4)) {
  m_robotDrive.arcadeDrive(-0.7, pidOut2);
}


}//Fin du teleop.periodic


}













   
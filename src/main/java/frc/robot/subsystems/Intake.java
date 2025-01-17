// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.ControlModeValue;
// import com.ctre.phoenix6.*;

// public class Intake extends SubsystemBase {
//     private static DoubleSolenoid intakeSolenoid;
//     private static TalonFX intakeMotor;
//     private static TalonFX indexMotor;

//     public Intake() {
//         intakeMotor = new TalonFX(19, "Canivore1");
//         // indexMotor = new TalonFX(3); indexer doesn't exist
//     }

//     // public void indexForward() {
//     // indexMotor.set(0.3);
//     // }

//     // public void indexReverse() {
//     // indexMotor.set(-0.3);
//     // }

//     public void intake() {
//         intakeMotor.set(-0.1);
//     }

//     public void outake() {
//         intakeMotor.set(0.4);
//     }

//     // public void stopIndex() {
//     // indexMotor.set(0);
//     // }
//     public void stopIntake() {
//         intakeMotor.set(0);
//     }
//     // public void stopIntakeMotors() {
//     // stopIntake();
//     // stopIndex();
//     // }
//     // public void intakeIndexForward(){
//     // intakeMotor.set(0.5);
//     // indexMotor.set(0.3);
//     // }
//     // public void outakeIndexReverse(){
//     // intakeMotor.set(-0.5);
//     // indexMotor.set(-0.3);
//     // }
// }

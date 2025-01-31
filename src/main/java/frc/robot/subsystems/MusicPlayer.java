package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;

public class MusicPlayer {
    Orchestra m_orchestra;
    public MusicPlayer(CommandSwerveDrivetrain drivetrain){
        m_orchestra=new Orchestra(getInstruments(drivetrain), "Rickroll.chrp");
    }
    
    public ArrayList<ParentDevice> getInstruments(CommandSwerveDrivetrain drivetrain){
        ArrayList<ParentDevice> instruments = new ArrayList<ParentDevice>();
        for(SwerveModule module:drivetrain.getModules()){
            instruments.add((TalonFX) module.getDriveMotor());
            instruments.add((TalonFX) module.getSteerMotor());
        }
        return instruments;
    }
    public void toggleMusic(){
        StatusCode status;
        if(m_orchestra.isPlaying()){
            status=m_orchestra.pause();
        }
        else{
            status=m_orchestra.play();
        }
    }
}

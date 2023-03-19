package frc.robot;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.AutoConstants.Objective;
import frc.robot.Constants.AutoConstants.Position;

public class Dashboard {
    private final ShuffleboardTab m_shuffleboardTab;
    private final String SHUFFLEBOARD_TAB_NAME = "Competition";
    private static SendableChooser<Objective> m_autoObjective = new SendableChooser<>();
    private static SendableChooser<Position> m_autoPosition = new SendableChooser<>();

    private final UsbCamera m_cameraView;
    private final int CAMERA_RES_HEIGHT = 30;
    private final int CAMERA_RES_WIDTH = 40;
    private final int CAMERA_FPS = 40;

    private final ComplexWidget m_CameraComplexWidget;
    private final ComplexWidget m_complexWidgetObjective;
    private final ComplexWidget m_complexWidgetPosition;
    private final GenericEntry m_pitch;

    private final GenericEntry m_isGrabberReset;
    
    public Dashboard() {
        m_shuffleboardTab =  Shuffleboard.getTab(SHUFFLEBOARD_TAB_NAME);
        Shuffleboard.selectTab(SHUFFLEBOARD_TAB_NAME);

        m_cameraView = CameraServer.startAutomaticCapture(0); // TODO add constants
        m_cameraView.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
        m_cameraView.setFPS(CAMERA_FPS);
        m_cameraView.setResolution(CAMERA_RES_WIDTH, CAMERA_RES_HEIGHT);

        m_CameraComplexWidget = m_shuffleboardTab.add("Camera View", m_cameraView)
            .withWidget(BuiltInWidgets.kCameraStream)
            .withSize(13, 11)
            .withPosition(0, 0);

        // objectives
        for (AutoConstants.Objective o : Objective.values()) {
            m_autoObjective.addOption(o.getDashboardDescript(), o);
        }
        m_autoObjective.setDefaultOption(Objective.Default.getDashboardDescript(), Objective.Default);
        m_complexWidgetObjective = m_shuffleboardTab.add( "AutoObjective", m_autoObjective)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withSize(4, 4)
            .withPosition(13, 0);

        // position
        for (AutoConstants.Position p : Position.values()) {
            m_autoPosition.addOption(p.getDashboardDescript(), p);
        }
        m_autoPosition.setDefaultOption(Position.Default.getDashboardDescript(), Position.Default);

            
        m_complexWidgetPosition = m_shuffleboardTab.add( "AutoPosition", m_autoPosition)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withSize(4, 4)
            .withPosition(17, 0);

        m_pitch = m_shuffleboardTab.add("Pitch", 0 )
            .withWidget(BuiltInWidgets.kTextView)
            .withSize(2, 2)
            .withPosition(13, 4)
            .getEntry();
        m_isGrabberReset = m_shuffleboardTab.add("Grabber reset", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withSize(3, 4)
            .withPosition(13, 8)
            .getEntry();

    } //end constructor

    //TODO place AUTO OBJECTIVE code here so we can choose our auto path
    public Objective getObjective() {
        return m_autoObjective.getSelected();
    }
    
    public void setPitchEntry(double value){
        m_pitch.setValue(value);
    } 

    public Position getPosition() {
        return m_autoPosition.getSelected();
    }

    public void setIfGrabberReset(boolean isReset) {
        m_isGrabberReset.setBoolean(isReset);
    }

}
 
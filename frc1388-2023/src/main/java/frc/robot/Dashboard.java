package frc.robot;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.Objective;

public class Dashboard {
    private final ShuffleboardTab m_shuffleboardTab;
    private final String SHUFFLEBOARD_TAB_NAME = "Competition"; //TODO maybe? put this in constants
    private static SendableChooser<Objective> m_autoObjective = new SendableChooser<>();

    private final UsbCamera m_cameraLarge;
    private final int CAMERA_LARGE_RES_HEIGHT = 480;
    private final int CAMERA_LARGE_RES_WIDTH = 640;
    private final int CAMERA_LARGE_FPS = 60;

    private final UsbCamera m_cameraSmall;
    private final int CAMERA_SMALL_RES_HEIGHT = 120;
    private final int CAMERA_SMALL_RES_WIDTH = 160;
    private final int CAMERA_SMALL_FPS = 10;

    // private final VideoSink m_videoSinkLargeCamera;
    // private final VideoSink m_videoSinkSmallCamera;

    private final ComplexWidget m_largeCameraComplexWidget;
    private final ComplexWidget m_smallCameraComplexWidget;
    private final ComplexWidget m_complexWidgetObjective;

    private final int autonChooserWidth = 5;
    private final int autonChooserHeight = 5;
    private final int autonChooserColumnIndex = 15; //where it is on shuffleboard
    private final int autonChooserRowIndex = 0; //where it is on shuffleboard
    
    public Dashboard() {
        
        System.out.println("**********************************\n\n\n\n\n\n\n\ndashboard\n\n\n\n\n\n\n\n\n****************************************");
        m_shuffleboardTab =  Shuffleboard.getTab(SHUFFLEBOARD_TAB_NAME);
        Shuffleboard.selectTab(SHUFFLEBOARD_TAB_NAME);

        m_cameraLarge = CameraServer.startAutomaticCapture(0); // TODO add constants
        m_cameraLarge.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
        m_cameraLarge.setFPS(CAMERA_LARGE_FPS);
        m_cameraLarge.setResolution(CAMERA_LARGE_RES_WIDTH, CAMERA_LARGE_RES_HEIGHT);
        // m_videoSinkLargeCamera = CameraServer.getServer();
        // m_videoSinkLargeCamera.setSource(m_cameraLarge);

        m_cameraSmall = CameraServer.startAutomaticCapture(1); // TODO add constantss
        m_cameraSmall.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
        m_cameraSmall.setFPS(CAMERA_SMALL_FPS);
        m_cameraSmall.setResolution(CAMERA_SMALL_RES_WIDTH, CAMERA_SMALL_RES_HEIGHT);
        // m_videoSinkLargeCamera.setSource(m_cameraSmall);


        m_largeCameraComplexWidget = m_shuffleboardTab.add("primary camera view", m_cameraLarge)
            .withWidget(BuiltInWidgets.kCameraStream)
            .withSize(4,4)
            .withPosition(0, 0);

        m_smallCameraComplexWidget = m_shuffleboardTab.add("secondary camera view", m_cameraSmall)
            .withWidget(BuiltInWidgets.kCameraStream)
            .withSize(4, 4)
            .withPosition(10, 0);

            for (Constants.Objective o: Objective.values()) {
                m_autoObjective.addOption(o.getDashboardDescript(), o);
            }

            m_complexWidgetObjective = m_shuffleboardTab.add( "AutoObjective", m_autoObjective)
            .withWidget(BuiltInWidgets.kSplitButtonChooser)
            .withSize(autonChooserWidth, autonChooserHeight)
            .withPosition(autonChooserColumnIndex, autonChooserRowIndex);
            

    } //end constructor

    //TODO place AUTO OBJECTIVE code here so we can choose our auto path
    public Objective getObjective() {
        return m_autoObjective.getSelected();
    } 

}

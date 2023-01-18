package frc.robot;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Dashboard {
    private final ShuffleboardTab m_shuffleboardTab;
    private final String SHUFFLEBOARD_TAB_NAME = "Competition"; //TODO maybe? put this in constants

    private final UsbCamera m_camera;
    private final int CAMERA_RES_HEIGHT = 480;
    private final int CAMERA_RES_WIDTH = 640;
    private final int CAMERA_FPS = 60;

    private final VideoSink m_videoSink;

    private final ComplexWidget m_cameraComplexWidget;

    public Dashboard() {
        System.out.println("\n\n\n\n\n\n\n\ndashboard\n\n\n\n\n\n\n\n\n");
        m_shuffleboardTab =  Shuffleboard.getTab(SHUFFLEBOARD_TAB_NAME);
        Shuffleboard.selectTab(SHUFFLEBOARD_TAB_NAME);

        m_camera = CameraServer.startAutomaticCapture(0); // TODO add constants
        m_camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
        m_camera.setFPS(CAMERA_FPS);
        m_camera.setResolution(CAMERA_RES_WIDTH, CAMERA_RES_HEIGHT);

        m_videoSink = CameraServer.getServer();
        m_videoSink.setSource(m_camera);

        m_cameraComplexWidget = m_shuffleboardTab.add("Camera view", m_videoSink.getSource())
            .withWidget(BuiltInWidgets.kCameraStream)
            .withSize(15, 14)
            .withPosition(0, 0);

        //TODO place AUTO POSITION code here so we can choose our auto path
        
    } // end constructor
}

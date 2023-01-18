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

public class Dashboard {
    private final ShuffleboardTab m_shuffleboardTab;
    private final String SHUFFLEBOARD_TAB_NAME = "Competition"; //TODO maybe? put this in constants

    private final UsbCamera m_cameraLarge;
    private final int CAMERA_LARGE_RES_HEIGHT = 480;
    private final int CAMERA_LARGE_RES_WIDTH = 640;
    private final int CAMERA_LARGE_FPS = 60;

    private final UsbCamera m_cameraSmall;
    private final int CAMERA_SMALL_RES_HEIGHT = 120;
    private final int CAMERA_SMALL_RES_WIDTH = 480;
    private final int CAMERA_SMALL_FPS = 10;

    private final VideoSink m_videoSinkLargeCamera;
    private final VideoSink m_videoSinkSmallCamera;

    private final ComplexWidget m_largeCameraComplexWidget;
    private final ComplexWidget m_smallCameraComplexWidget;
    
    public Dashboard() {
        
        System.out.println("**********************************\n\n\n\n\n\n\n\ndashboard\n\n\n\n\n\n\n\n\n****************************************");
        m_shuffleboardTab =  Shuffleboard.getTab(SHUFFLEBOARD_TAB_NAME);
        Shuffleboard.selectTab(SHUFFLEBOARD_TAB_NAME);

        m_cameraLarge = CameraServer.startAutomaticCapture(0); // TODO add constants
        m_cameraLarge.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
        m_cameraLarge.setFPS(CAMERA_LARGE_FPS);
        m_cameraLarge.setResolution(CAMERA_LARGE_RES_WIDTH, CAMERA_LARGE_RES_HEIGHT);
        m_videoSinkLargeCamera = CameraServer.getServer();
        m_videoSinkLargeCamera.setSource(m_cameraLarge);

        m_cameraSmall = CameraServer.startAutomaticCapture(1); // TODO add constants
        m_cameraSmall.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
        m_cameraSmall.setFPS(CAMERA_SMALL_FPS);
        m_cameraSmall.setResolution(CAMERA_SMALL_RES_WIDTH, CAMERA_SMALL_RES_HEIGHT);
        m_videoSinkSmallCamera = CameraServer.getServer();
        m_videoSinkSmallCamera.setSource(m_cameraSmall);


        m_largeCameraComplexWidget = m_shuffleboardTab.add("primary camera view", m_videoSinkLargeCamera.getSource())
            .withWidget(BuiltInWidgets.kCameraStream)
            .withSize(15, 14)
            .withPosition(0, 0);

        m_smallCameraComplexWidget = m_shuffleboardTab.add("secondary camera view", m_videoSinkLargeCamera.getSource())
            .withWidget(BuiltInWidgets.kCameraStream)
            .withSize(5, 5)
            .withPosition(16, 0);
    } // end constructor
}

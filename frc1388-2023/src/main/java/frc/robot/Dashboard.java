package frc.robot;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.Objective;
import frc.robot.Constants.Position;
import frc.robot.subsystems.GyroSubsystem;

public class Dashboard {
    private final ShuffleboardTab m_shuffleboardTab;
    private final String SHUFFLEBOARD_TAB_NAME = "Competition"; //TODO maybe? put this in constants
    private static SendableChooser<Objective> m_autoObjective = new SendableChooser<>();
    private static SendableChooser<Position> m_autoPosition = new SendableChooser<>();

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
    private final ComplexWidget m_complexWidgetPosition;
    private final GenericEntry m_pitch;

    //private final int autonChooserWidth = 5;
    //private final int autonChooserHeight = 5;
    //private final int autonChooserColumnIndex = 15; //where it is on shuffleboard
    //private final int autonChooserRowIndex = 1; //where it is on shuffleboard
    
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
            .withSize(2,2)
            .withPosition(0, 0);

        m_smallCameraComplexWidget = m_shuffleboardTab.add("secondary camera view", m_cameraSmall)
            .withWidget(BuiltInWidgets.kCameraStream)
            .withSize(4, 4)
            .withPosition(10, 0);

            for (Constants.Objective o: Objective.values()) {
                m_autoObjective.addOption(o.getDashboardDescript(), o);
            }

            for (Constants.Position p: Position.values()) {
                m_autoPosition.addOption(p.getDashboardDescript(), p);
            }

        m_complexWidgetObjective = m_shuffleboardTab.add( "AutoObjective", m_autoObjective)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withSize(4, 4)
            .withPosition(10, 7);

        m_pitch = m_shuffleboardTab.add("Pitch", 0 )
            .withWidget(BuiltInWidgets.kTextView)
            .withSize(2, 2)
            .withPosition(16, 0)
            .getEntry();

        m_complexWidgetPosition = m_shuffleboardTab.add( "AutoPosition", m_autoPosition)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withSize(4, 4)
            .withPosition(16, 7);


        //DELETEME: testing autobalance pid loop
        SmartDashboard.putNumber("F", DriveTrainConstants.GAINS_VELOCITY_F);
        SmartDashboard.putNumber("P", DriveTrainConstants.GAINS_VELOCITY_P);
        SmartDashboard.putNumber("I", DriveTrainConstants.GAINS_VELOCITY_I);
        SmartDashboard.putNumber("D", DriveTrainConstants.GAINS_VELOCITY_D);
        SmartDashboard.putNumber("speed", 6);

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

}
 
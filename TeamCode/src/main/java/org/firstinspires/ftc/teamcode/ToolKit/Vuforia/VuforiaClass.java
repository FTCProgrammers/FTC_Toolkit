package org.firstinspires.ftc.teamcode.ToolKit.Vuforia;
import com.vuforia.HINT;
import com.vuforia.Vuforia;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.ToolKit.Utilities.Constants;

public class VuforiaClass {
    private VuforiaLocalizer.Parameters vuforiaSettings;
    private VuforiaTrackables targets;
    private CameraSide cameraSide;
    private PhoneOrientation orientation;
    private VuforiaTrackableDefaultListener listener;

    public VuforiaClass(CameraSide cameraSide, PhoneOrientation orientation, int maxSimultaneousImageTargets, boolean showCameraFeedbackOnPhone) {
        this.cameraSide = cameraSide;
        this.orientation = orientation;
        if(showCameraFeedbackOnPhone) {
            vuforiaSettings = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        } else {
            vuforiaSettings = new VuforiaLocalizer.Parameters();
        }

        switch(cameraSide) {
            case SCREEN:
                vuforiaSettings.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
                break;
            case BACK:
                vuforiaSettings.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
                break;
        }

        vuforiaSettings.vuforiaLicenseKey = Constants.LICENSE;
        vuforiaSettings.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, maxSimultaneousImageTargets);
    }



    public void init() {
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(vuforiaSettings);

        targets = vuforia.loadTrackablesFromAsset("RelicVuMark");

        targets.get(0).setName("Crypto Key");

        targets.activate();

        listener = (VuforiaTrackableDefaultListener) targets.get(0).getListener();
    }


    public boolean canSeeTarget() {
        return listener.getPose() != null;
    }


    public CryptoColumn decodeTarget() {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(targets.get(0));
        switch(vuMark) {
            case UNKNOWN:
                return CryptoColumn.UNKNOWN;
            case LEFT:
                return CryptoColumn.LEFT;
            case CENTER:
                return CryptoColumn.CENTER;
            case RIGHT:
                return CryptoColumn.RIGHT;
            default:
                return CryptoColumn.UNKNOWN;
        }
    }



    public CameraSide getCameraSide() {
        return cameraSide;
    }

    public PhoneOrientation getOrientation() {
        return orientation;
    }
}
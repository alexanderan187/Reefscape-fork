package frc.robot.vision;

import static frc.robot.Constants.FieldK.kTagLayout;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;

public class VisionSim {
    private final VisionSystemSim m_photonVisionSim;

    public VisionSim() {
        m_photonVisionSim = new VisionSystemSim("main");
        m_photonVisionSim.addAprilTags(kTagLayout);
    }

    /**
     * <p> This method adds a given PhotonCameraSim to the vision simulation to
     *  incorporate simulation of this camera into vision simulation.
     * @param photonCameraSim The PhotonCameraSim object for the camera you'd like to add
     * @param roboToCam The Transform3d from the robot pose to the camera location
     */
    public void addCamera(PhotonCameraSim photonCameraSim, Transform3d roboToCam) {
        m_photonVisionSim.addCamera(photonCameraSim, roboToCam);
    }

    /**
     * <p> This method should contain everything that needs to be called every
     *  sim loop. This method then needs to be called every sim loop
     *  from Robot.java as it is not a subsystem
     * @param robotSimPose Pose2d object that represents the robots current
     *  simulated pose
     */
    public void simulationPeriodic(Pose2d robotSimPose) {
        m_photonVisionSim.update(robotSimPose);
    }

    // this method isn't used anywhere but it would probably make sense
    // to call this whenever you use resetPose on the drivetrain to
    // update the vision simulation. no idea. it seems to work alright
    // without it
    public void resetSimPose(Pose2d pose) {
        if (Robot.isSimulation()) {
            m_photonVisionSim.resetRobotPose(pose);
        }
    }

    /**
     * @return A Field2d object representing all of the debug
     *  information for vision simulation (april tags, etc.)
     */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) {
            return new Field2d();
        }

        return m_photonVisionSim.getDebugField();
    }
}

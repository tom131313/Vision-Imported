package frc.robot;

import java.io.IOException;
import java.lang.invoke.MethodHandles;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * Easy access to the WPILib AprilTag Field Layout
 */
public class AprilTagsLocations
 {      
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    private static AprilTagFieldLayout aprilTagFieldLayout;
    private static NetworkTable tagsTable = NetworkTableInstance.getDefault().getTable("AprilTagsLocations");

    // AprilTagsLocations()
    static {
         
        final boolean CustomTagLayout = false; // true is use custom deploy of layout

        try {
            if(CustomTagLayout)
                aprilTagFieldLayout = new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/2024-crescendo.json"); // custom file example
            else
                aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        } catch (IOException e) {
            e.printStackTrace();
            aprilTagFieldLayout = null;
        }

        List<StructPublisher<Pose3d>> publishTagPose = new ArrayList<>(getTagCount());
        
        Consumer<AprilTag> initializeTagPoses = tag ->
        {
            System.out.format("%s %6.1f, %6.1f, %6.1f [degrees]%n",
                tag.toString(),
                Units.radiansToDegrees(tag.pose.getRotation().getX()),
                Units.radiansToDegrees(tag.pose.getRotation().getY()),
                Units.radiansToDegrees(tag.pose.getRotation().getZ()));
                // assuming there is no tag 0 so indexing starts with tag 1 in list position 0. Ugh!
                var tagPosePublisher = tagsTable.getStructTopic("tagPose3D_" + tag.ID, Pose3d.struct).publish();
                publishTagPose.add(tagPosePublisher);
                publishTagPose.get(tag.ID-1).set(tag.pose); // no tag 0 so index back 1; ouch! that sequencing trick hurts!
        };

        System.out.println(aprilTagFieldLayout.getTags().size() + " Tags on file");
        aprilTagFieldLayout.getTags().forEach(initializeTagPoses);
    }

    public static int getTagCount()
    {
        return aprilTagFieldLayout.getTags().size();
    }

    public static List<AprilTag> getTagsLocations()
    {
        return aprilTagFieldLayout.getTags();
    } 
    
    public static Pose3d getTagLocation(int tagID)
    {
        return aprilTagFieldLayout.getTags().get(tagID-1).pose; // no tag 0 so index back one; ouch! that sequencing trick hurts!
    }
}

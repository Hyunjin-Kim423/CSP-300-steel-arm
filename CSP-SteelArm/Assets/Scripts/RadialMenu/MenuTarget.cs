using System.Collections.Generic;
using RosMessageTypes.Yolo;
using UnityEngine;

// What the radial menu shows about the selected object, and where that object is.
public class MenuTarget
{
    public string name;          // YOLO class, e.g. "cup"
    public float confidence;     // YOLO score
    public string[] actions;     // robot actions from yolo_dectection_list.json (null = not registered)
    public Color color;          // green (registered) / red (not registered)
    public Vector3 center;       // object center in the world
    public float halfWidth;      // half of the object's width in meters (keeps the menu from covering it)
    public Vector3[] outline;    // outline points in the world; the connector ends on the one nearest the menu
    public DetectionMsg detection;

    public bool Registered => actions != null;

    public static MenuTarget From(DetectedObject obj)
    {
        var outline = new List<Vector3>();
        for (int i = 0; i < obj.partCount; i++)
            outline.AddRange(obj.parts[i].current);

        return new MenuTarget
        {
            name = obj.detection.class_name,
            confidence = (float)obj.detection.score,
            actions = RobotActionRegistry.GetActions(obj.detection.class_name),
            color = obj.color,
            center = obj.Center,
            halfWidth = Vector3.Distance(obj.corners[0], obj.corners[3]) * 0.5f,
            outline = outline.ToArray(),
            detection = obj.detection
        };
    }

    // An object from the cube visualizer (its silhouette, or its box). color: its highlight color (registered / not).
    public static MenuTarget From(ObjectCube cube, Color color)
    {
        var target = new MenuTarget
        {
            name = cube.detection.class_name,
            confidence = (float)cube.detection.score,
            actions = RobotActionRegistry.GetActions(cube.detection.class_name),
            color = color,
            detection = cube.detection
        };
        if (!cube.drawBox && cube.silhouette.HasShape)
        {
            var outline = new List<Vector3>();
            cube.silhouette.GetWorldOutline(outline);
            target.center = cube.silhouette.Position;
            target.halfWidth = cube.silhouette.HalfWidth;
            target.outline = outline.ToArray();
        }
        else
        {
            var corners = new Vector3[8];
            cube.GetDrawnCorners(corners);
            target.center = cube.DrawnCenter;
            target.halfWidth = cube.DrawnSize.x * 0.5f;
            target.outline = corners;
        }
        return target;
    }
}

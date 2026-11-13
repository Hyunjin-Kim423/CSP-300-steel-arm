using System.Collections.Generic;
using RosMessageTypes.Yolo;
using UnityEngine;

// What the radial menu shows about the selected object, and where that object is.
public class MenuTarget
{
    public string name;          // YOLO class, e.g. "cup"
    public float confidence;     // YOLO score
    public string[] actions;     // robot actions from yolo_dectection_list.json (null = not registered)
    public Color color;          // highlight color: registered / not registered
    public Vector3 center;       // where the object's silhouette is drawn (world)
    public float halfWidth;      // half of the silhouette's width in meters (keeps the menu from covering it)
    public Vector3[] outline;    // silhouette points in the world; the connector ends on the one nearest the menu
    public DetectionMsg detection;

    public bool Registered => actions != null;

    // color: the object's highlight color.
    public static MenuTarget From(ObjectCube cube, Color color)
    {
        var outline = new List<Vector3>();
        cube.silhouette.GetWorldOutline(outline);
        return new MenuTarget
        {
            name = cube.detection.class_name,
            confidence = (float)cube.detection.score,
            actions = RobotActionRegistry.GetActions(cube.detection.class_name),
            color = color,
            center = cube.silhouette.Position,
            halfWidth = cube.silhouette.HalfWidth,
            outline = outline.ToArray(),
            detection = cube.detection
        };
    }
}

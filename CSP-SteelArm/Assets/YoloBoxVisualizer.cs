using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.InputSystem;
using Unity.Robotics.ROSTCPConnector;
using Unity.XR.CoreUtils;
using RosMessageTypes.Yolo;
using EnvironmentRaycastManager = Meta.XR.EnvironmentRaycastManager;
using PassthroughCameraAccess = Meta.XR.PassthroughCameraAccess;

[RequireComponent(typeof(CameraImagePublisher))]
public class YoloBoxVisualizer : MonoBehaviour
{
    class Box
    {
        public LineRenderer line;
        public readonly Vector3[] corners = new Vector3[4];
        public string text;
        public float depth;
        public Vector3 cameraPosition;
    }

    class Controller
    {
        public readonly InputAction trigger;
        public readonly InputAction position;
        public readonly InputAction rotation;

        public Controller(string hand)
        {
            trigger = new InputAction(type: InputActionType.Button, binding: $"<XRController>{{{hand}}}/{{TriggerButton}}");
            position = new InputAction(binding: $"<XRController>{{{hand}}}/pointerPosition");
            rotation = new InputAction(binding: $"<XRController>{{{hand}}}/pointerRotation");
            trigger.Enable();
            position.Enable();
            rotation.Enable();
        }

        public void Dispose()
        {
            trigger.Dispose();
            position.Dispose();
            rotation.Dispose();
        }
    }

    public string topicName = "/yolo/detections";
    [Tooltip("Depth (meters) used when the environment depth raycast misses.")]
    public float fallbackDistance = 1.0f;
    public float lineWidth = 0.004f;
    public Color boxColor = Color.green;
    [Tooltip("Label text height in meters for an object 1 m away; scales with distance.")]
    public float labelHeight = 0.03f;
    [Tooltip("How long the label stays after pulling the trigger (seconds).")]
    public float labelDuration = 3f;
    [Tooltip("Boxes are hidden if no new detections arrive within this many seconds.")]
    public float boxLifetime = 2f;

    CameraImagePublisher publisher;
    EnvironmentRaycastManager environmentRaycast;
    XROrigin xrOrigin;
    Controller[] controllers;
    Material lineMaterial;
    TextMeshPro label;
    readonly List<Box> boxes = new List<Box>();
    float lastDetectionTime;
    float labelHideTime;

    void Start()
    {
        publisher = GetComponent<CameraImagePublisher>();
        xrOrigin = FindAnyObjectByType<XROrigin>();
        controllers = new[] { new Controller("RightHand"), new Controller("LeftHand") };
        lineMaterial = new Material(Shader.Find("Sprites/Default"));

        label = new GameObject("YoloLabel", typeof(RectTransform)).AddComponent<TextMeshPro>();
        label.rectTransform.pivot = Vector2.zero;
        label.rectTransform.sizeDelta = Vector2.zero;
        label.alignment = TextAlignmentOptions.BottomLeft;
        label.textWrappingMode = TextWrappingModes.NoWrap;
        label.color = boxColor;
        label.outlineWidth = 0.2f;
        label.outlineColor = new Color32(0, 0, 0, 255);
        label.gameObject.SetActive(false);

        if (EnvironmentRaycastManager.IsSupported)
        {
            OVRPermissionsRequester.Request(new[] { OVRPermissionsRequester.Permission.Scene });
            environmentRaycast = FindAnyObjectByType<EnvironmentRaycastManager>();
            if (environmentRaycast == null)
                environmentRaycast = gameObject.AddComponent<EnvironmentRaycastManager>();
        }

        ROSConnection.GetOrCreateInstance().Subscribe<DetectionArrayMsg>(topicName, OnDetections);
    }

    void OnDestroy()
    {
        foreach (var controller in controllers)
            controller.Dispose();
    }

    void Update()
    {
        if (Time.time - lastDetectionTime > boxLifetime)
            HideBoxesFrom(0);
        if (Time.time > labelHideTime)
            label.gameObject.SetActive(false);

        foreach (var controller in controllers)
        {
            if (controller.trigger.WasPressedThisFrame())
                ShowLabelForPointedBox(AimRay(controller));
        }
    }

    Ray AimRay(Controller controller)
    {
        // Controller poses are reported in tracking space, relative to the XR Origin's camera offset.
        Transform trackingSpace = xrOrigin.CameraFloorOffsetObject.transform;
        Vector3 position = trackingSpace.TransformPoint(controller.position.ReadValue<Vector3>());
        Quaternion rotation = trackingSpace.rotation * controller.rotation.ReadValue<Quaternion>();
        return new Ray(position, rotation * Vector3.forward);
    }

    void ShowLabelForPointedBox(Ray ray)
    {
        Box pointed = null;
        float nearest = float.MaxValue;
        foreach (var box in boxes)
        {
            if (box.line.gameObject.activeSelf && RayHitsBox(ray, box.corners, out float distance) && distance < nearest)
            {
                pointed = box;
                nearest = distance;
            }
        }

        if (pointed == null)
        {
            label.gameObject.SetActive(false);
            return;
        }

        Vector3 topLeft = pointed.corners[1];
        label.text = pointed.text;
        label.fontSize = labelHeight * pointed.depth * 10f;
        label.transform.SetPositionAndRotation(topLeft, Quaternion.LookRotation(topLeft - pointed.cameraPosition));
        label.gameObject.SetActive(true);
        labelHideTime = Time.time + labelDuration;
    }

    static bool RayHitsBox(Ray ray, Vector3[] corners, out float distance)
    {
        if (!new Plane(corners[0], corners[1], corners[2]).Raycast(ray, out distance))
            return false;
        Vector3 hit = ray.GetPoint(distance) - corners[0];
        Vector3 up = corners[1] - corners[0];
        Vector3 right = corners[3] - corners[0];
        float u = Vector3.Dot(hit, right) / right.sqrMagnitude;
        float v = Vector3.Dot(hit, up) / up.sqrMagnitude;
        return u >= 0f && u <= 1f && v >= 0f && v <= 1f;
    }

    void OnDetections(DetectionArrayMsg msg)
    {
        var cameraAccess = publisher.CameraAccess;
        if (!cameraAccess.IsPlaying) return;
        if (!publisher.TryGetFrame(msg.header.stamp, out var frame)) return;

        Pose pose = frame.cameraPose;
        Vector3 forward = pose.rotation * Vector3.forward;
        float w = frame.imageSize.x;
        float h = frame.imageSize.y;
        int count = 0;
        foreach (var det in msg.detections)
        {
            var center = det.bbox.center.position;
            var size = det.bbox.size;
            // YOLO pixels have a top-left origin; the camera viewport has a bottom-left origin.
            float left = (float)(center.x - size.x / 2) / w;
            float right = (float)(center.x + size.x / 2) / w;
            float top = 1f - (float)(center.y - size.y / 2) / h;
            float bottom = 1f - (float)(center.y + size.y / 2) / h;

            Ray centerRay = cameraAccess.ViewportPointToRay(new Vector2((left + right) / 2, (top + bottom) / 2), pose);
            float depth = DepthAlong(centerRay, forward);

            var box = GetBox(count++);
            box.corners[0] = PointAtDepth(cameraAccess, pose, forward, left, bottom, depth);
            box.corners[1] = PointAtDepth(cameraAccess, pose, forward, left, top, depth);
            box.corners[2] = PointAtDepth(cameraAccess, pose, forward, right, top, depth);
            box.corners[3] = PointAtDepth(cameraAccess, pose, forward, right, bottom, depth);
            box.line.SetPositions(box.corners);
            box.text = $"{det.class_name} {det.score:0.00}";
            box.depth = depth;
            box.cameraPosition = pose.position;
        }

        HideBoxesFrom(count);
        lastDetectionTime = Time.time;
    }

    // Returns the object's distance along the camera's optical axis.
    float DepthAlong(Ray ray, Vector3 forward)
    {
        if (environmentRaycast != null && environmentRaycast.Raycast(ray, out var hit))
            return Vector3.Dot(hit.point - ray.origin, forward);
        return fallbackDistance;
    }

    static Vector3 PointAtDepth(PassthroughCameraAccess cameraAccess, Pose pose, Vector3 forward, float x, float y, float depth)
    {
        Ray ray = cameraAccess.ViewportPointToRay(new Vector2(x, y), pose);
        return ray.origin + ray.direction * (depth / Vector3.Dot(ray.direction, forward));
    }

    Box GetBox(int index)
    {
        if (index == boxes.Count)
        {
            var line = new GameObject("YoloBox").AddComponent<LineRenderer>();
            line.useWorldSpace = true;
            line.loop = true;
            line.positionCount = 4;
            line.widthMultiplier = lineWidth;
            line.material = lineMaterial;
            line.startColor = boxColor;
            line.endColor = boxColor;
            boxes.Add(new Box { line = line });
        }
        boxes[index].line.gameObject.SetActive(true);
        return boxes[index];
    }

    void HideBoxesFrom(int index)
    {
        for (int i = index; i < boxes.Count; i++)
            boxes[i].line.gameObject.SetActive(false);
    }
}

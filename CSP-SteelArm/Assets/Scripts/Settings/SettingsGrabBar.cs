using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.InputSystem;
using UnityEngine.UI;
using UnityEngine.XR.Interaction.Toolkit.UI;

// The bar under the settings panel (like the Quest OS windows): point at it, hold the trigger and move the ray to
// carry the panel; while holding, push a thumbstick forward / back to move it farther / closer.
// The panel keeps facing the player while it is carried.
public class SettingsGrabBar : MonoBehaviour, IPointerEnterHandler, IPointerExitHandler,
    IInitializePotentialDragHandler, IBeginDragHandler, IDragHandler, IEndDragHandler
{
    [Tooltip("What is carried (the settings panel root).")]
    public Transform panel;
    [Tooltip("The visible bar.")]
    public Image bar;

    [Header("Look")]
    public Color normalColor = new Color(1f, 1f, 1f, 0.35f);
    public Color hoverColor = new Color(1f, 1f, 1f, 0.75f);
    public Color dragColor = Color.white;
    public Vector2 normalSize = new Vector2(160f, 10f);
    public Vector2 activeSize = new Vector2(200f, 12f);

    [Header("Carry")]
    [Tooltip("Distance limits from the controller while carrying (meters).")]
    public float minDistance = 0.3f;
    public float maxDistance = 2f;
    [Tooltip("Thumbstick forward / back speed (m/s).")]
    public float stickSpeed = 0.8f;
    public float stickDeadZone = 0.25f;
    [Tooltip("How quickly the panel catches up with the ray. Higher = snappier.")]
    public float followSpeed = 18f;

    PointerEventData dragData;
    float grabDistance;
    Vector3 localOffset;    // panel center relative to the grabbed point, in panel space
    bool hovered;
    InputAction leftStick;
    InputAction rightStick;

    public bool IsDragging => dragData != null;

    void Awake()
    {
        leftStick = new InputAction(binding: "<XRController>{LeftHand}/{Primary2DAxis}");
        rightStick = new InputAction(binding: "<XRController>{RightHand}/{Primary2DAxis}");
        ApplyLook(1f);
    }

    void OnDisable()
    {
        dragData = null;
        hovered = false;
        leftStick.Disable();
        rightStick.Disable();
        ApplyLook(1f);
    }

    void OnDestroy()
    {
        leftStick?.Dispose();
        rightStick?.Dispose();
    }

    public void OnPointerEnter(PointerEventData eventData) => hovered = true;
    public void OnPointerExit(PointerEventData eventData) => hovered = false;

    // Start carrying right away, without the usual drag threshold.
    public void OnInitializePotentialDrag(PointerEventData eventData) => eventData.useDragThreshold = false;

    public void OnBeginDrag(PointerEventData eventData)
    {
        if (panel == null || !TryGetRay(eventData, out Ray ray))
            return;
        Vector3 grabPoint = eventData.pointerPressRaycast.isValid ? eventData.pointerPressRaycast.worldPosition
            : eventData.pointerCurrentRaycast.worldPosition;
        grabDistance = Mathf.Clamp(Vector3.Dot(grabPoint - ray.origin, ray.direction), minDistance, maxDistance);
        localOffset = Quaternion.Inverse(panel.rotation) * (panel.position - grabPoint);
        dragData = eventData;
        leftStick.Enable();
        rightStick.Enable();
    }

    public void OnDrag(PointerEventData eventData) { } // the panel is moved in Update, so the thumbstick works without moving the ray

    public void OnEndDrag(PointerEventData eventData)
    {
        dragData = null;
        leftStick.Disable();
        rightStick.Disable();
    }

    void Update()
    {
        ApplyLook(1f - Mathf.Exp(-15f * Time.deltaTime));
        if (dragData == null || !TryGetRay(dragData, out Ray ray))
            return;

        float stick = StickInput(leftStick) + StickInput(rightStick);
        grabDistance = Mathf.Clamp(grabDistance + stick * stickSpeed * Time.deltaTime, minDistance, maxDistance);

        Vector3 grabPoint = ray.origin + ray.direction * grabDistance;
        Camera cam = Camera.main;
        Vector3 viewer = cam != null ? cam.transform.position : ray.origin;
        Vector3 facing = grabPoint - viewer;
        if (facing.sqrMagnitude < 1e-4f)
            return;
        Quaternion rotation = Quaternion.LookRotation(facing, Vector3.up); // canvas forward points away from the player
        Vector3 position = grabPoint + rotation * localOffset;

        float t = 1f - Mathf.Exp(-followSpeed * Time.deltaTime);
        panel.SetPositionAndRotation(Vector3.Lerp(panel.position, position, t), Quaternion.Slerp(panel.rotation, rotation, t));
    }

    // The controller ray (XR), or the mouse ray in the editor.
    static bool TryGetRay(PointerEventData data, out Ray ray)
    {
        if (data is TrackedDeviceEventData tracked && tracked.rayPoints != null && tracked.rayPoints.Count >= 2)
        {
            Vector3 start = tracked.rayPoints[0];
            Vector3 direction = tracked.rayPoints[1] - start;
            if (direction.sqrMagnitude > 1e-8f)
            {
                ray = new Ray(start, direction.normalized);
                return true;
            }
        }
        Camera cam = data.pressEventCamera != null ? data.pressEventCamera : Camera.main;
        if (cam != null)
        {
            ray = cam.ScreenPointToRay(data.position);
            return true;
        }
        ray = default;
        return false;
    }

    float StickInput(InputAction stick)
    {
        Vector2 value = stick.ReadValue<Vector2>();
        if (Mathf.Abs(value.y) < stickDeadZone || Mathf.Abs(value.y) < Mathf.Abs(value.x))
            return 0f;
        return value.y;
    }

    void ApplyLook(float t)
    {
        if (bar == null)
            return;
        bool active = dragData != null || hovered;
        Color color = dragData != null ? dragColor : hovered ? hoverColor : normalColor;
        bar.color = Color.Lerp(bar.color, color, t);
        bar.rectTransform.sizeDelta = Vector2.Lerp(bar.rectTransform.sizeDelta, active ? activeSize : normalSize, t);
    }
}

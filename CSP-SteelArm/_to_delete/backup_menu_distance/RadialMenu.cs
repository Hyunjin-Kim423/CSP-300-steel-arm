using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

// Radial object menu (design A): the object's name and info in the center, robot actions on a half-ring,
// a close button at the top, and a curved line from the center to the nearest point of the object's outline.
//
// The action buttons sit on the half-ring AWAY from the object, so the line always leaves through the open side.
// When the menu moves to the other side of the object, the buttons slide across: each keeps its height, shrinks
// into the center and grows out on the other side (mirrorTowardObject).
//
// The menu floats beside the object and follows the player's gaze a little: it moves around the object (yaw only)
// toward where the player looks, but never more than maxFollowAngle away from the player->object direction,
// and never so close to that direction that it would cover the object.
//
// The prefab is built by SteelArm > Build Radial Menu Prefabs (Assets/Editor/RadialMenuPrefabBuilder.cs).
public class RadialMenu : MonoBehaviour
{
    [Header("Parts (filled in by the prefab builder)")]
    public RectTransform hub;
    public TMP_Text nameText;
    public TMP_Text infoText;
    public TMP_Text statusText;
    public RectTransform buttonRoot;
    public MenuButtonVisual actionButtonPrefab;
    public MenuButtonVisual closeButton;
    public LineRenderer connector;
    public LineRenderer anchorDot;

    [Header("Layout (canvas units)")]
    public float hubRadius = 64f;
    public float buttonRadius = 122f;
    [Tooltip("Angle between neighbouring action buttons (degrees).")]
    public float buttonSpacingAngle = 60f;
    [Tooltip("Largest arc the action buttons may cover (degrees); spacing shrinks to fit.")]
    public float maxButtonArc = 200f;
    [Tooltip("Direction of the close button from the center when the object is on the menu's left (degrees, 0 = right, 90 = up). Mirrored when the object is on the right.")]
    public float closeButtonAngle = 125f;

    [Header("Buttons switch sides with the object")]
    [Tooltip("Keep the action buttons on the side away from the object. Off = always on the right half.")]
    public bool mirrorTowardObject = true;
    [Tooltip("How long the buttons take to slide to the other side (seconds).")]
    public float mirrorDuration = 0.35f;
    [Tooltip("The object must be this far (canvas units) past the menu's center before the buttons switch sides.")]
    public float mirrorHysteresis = 24f;

    [Header("Follow the player's gaze")]
    [Tooltip("The menu follows where the player looks, but stays within this yaw angle (degrees) of the player->object direction.")]
    public float maxFollowAngle = 30f;
    [Tooltip("The menu stays at least this far to the side (degrees) so it does not cover the object. Grows automatically for wide objects.")]
    public float minSideAngle = 12f;
    [Tooltip("How quickly the menu catches up with the gaze. Lower = calmer.")]
    public float followSpeed = 3f;
    [Tooltip("How much closer to the player than the object the menu floats (meters).")]
    public float inFrontOfObject = 0.15f;
    public float minDistance = 0.35f;
    public float maxDistance = 1.2f;
    [Tooltip("Height of the menu relative to the object's center (meters).")]
    public float heightOffset = 0.05f;

    [Header("Connector")]
    public float lineWidth = 0.003f;
    public int curveSegments = 24;
    public float dotRadius = 0.008f;

    [Header("Animation")]
    public float duration = 0.45f;
    public float staggerDelay = 0.05f;

    static readonly Color DarkText = new Color(0.04f, 0.1f, 0.06f, 1f);
    static readonly Color CloseHover = new Color(0.94f, 0.27f, 0.27f, 1f);
    const int DotSegments = 16;

    static Material s_LineMaterial;

    MenuTarget target;
    Transform head;
    float yaw;
    int side = 1;
    bool isOpen;
    bool isClosing;
    readonly List<RectTransform> buttons = new List<RectTransform>();      // action buttons + close button
    readonly List<float> buttonAngles = new List<float>();                 // their directions for the "object on the left" layout
    float mirror = 1f;      // +1: buttons on the right half (object on the left), -1: mirrored; in between while sliding
    bool layoutLive;        // the open animation is done; Update keeps the layout
    Vector3 lastAnchor;
    Vector3[] curve;
    readonly Vector3[] dot = new Vector3[DotSegments];

    public bool IsClosing => isClosing;

    // ---------- open / close ----------

    public void Open(MenuTarget menuTarget)
    {
        target = menuTarget;
        head = Camera.main != null ? Camera.main.transform : null;
        var canvas = GetComponent<Canvas>();
        if (canvas != null && canvas.worldCamera == null)
            canvas.worldCamera = Camera.main;

        FillInfo();
        BuildButtons();
        SetUpLines();

        // Appear right away beside the object, on the side the player is looking toward,
        // with the buttons already on the side away from the object.
        if (head != null)
        {
            yaw = TargetYaw();
            PlaceMenu();
            UpdateConnector();
            mirror = MirrorTarget();
        }
        isOpen = true;
        StartCoroutine(OpenAnimation());
    }

    // Plays the closing animation, then destroys the menu. Safe to call more than once.
    public void Close()
    {
        if (isClosing)
            return;
        isClosing = true;
        layoutLive = false;
        StopAllCoroutines();
        foreach (var button in buttons)
        {
            if (button != null && button.TryGetComponent(out Button b))
            {
                b.onClick.RemoveAllListeners();
                b.interactable = false;
            }
        }
        StartCoroutine(CloseAnimation());
    }

    void FillInfo()
    {
        nameText.text = target.name;
        infoText.text = $"Confidence {target.confidence:0.00}";
        int count = target.Registered ? target.actions.Length : 0;
        statusText.text = target.Registered ? (count == 1 ? "1 action" : $"{count} actions") : "No actions";
        statusText.color = target.Registered ? Color.green : Color.red;
    }

    void BuildButtons()
    {
        buttons.Clear();
        buttonAngles.Clear();

        // Action buttons, spread evenly over the half-ring, top to bottom.
        string[] actions = target.actions ?? new string[0];
        int n = actions.Length;
        float spacing = n > 1 ? Mathf.Min(buttonSpacingAngle, maxButtonArc / (n - 1)) : 0f;
        for (int i = 0; i < n; i++)
        {
            float angle = ((n - 1) * 0.5f - i) * spacing;
            MenuButtonVisual button = Instantiate(actionButtonPrefab, buttonRoot);
            button.name = "Action " + actions[i];
            button.SetLabel(actions[i]);
            button.SetHoverColors(target.color, DarkText);
            string method = actions[i];
            button.Button.onClick.AddListener(() => RobotActionRegistry.ExecuteAction(method));
            AddButton((RectTransform)button.transform, angle);
        }

        // Close button (on the object's side, at the top): turns red on hover.
        closeButton.SetHoverColors(CloseHover, Color.white);
        closeButton.Button.onClick.AddListener(Close);
        AddButton((RectTransform)closeButton.transform, closeButtonAngle);
    }

    void AddButton(RectTransform button, float angleDegrees)
    {
        buttons.Add(button);
        buttonAngles.Add(angleDegrees);
    }

    // ---------- button side ----------

    // Position of button i for the current side. Mirroring only flips x, so each button keeps its height.
    Vector2 LayoutPosition(int i)
    {
        float rad = buttonAngles[i] * Mathf.Deg2Rad;
        return new Vector2(Mathf.Cos(rad) * buttonRadius * mirror, Mathf.Sin(rad) * buttonRadius);
    }

    // Buttons shrink while they cross the center, so they never pile up on top of it.
    float LayoutScale() => Mathf.SmoothStep(0f, 1f, Mathf.Abs(mirror));

    // +1 when the object is on the menu's left (buttons go right), -1 when it is on the right.
    float MirrorTarget()
    {
        if (!mirrorTowardObject)
            return 1f;
        float objectX = transform.InverseTransformPoint(lastAnchor).x; // canvas units, + = right
        if (objectX < -mirrorHysteresis)
            return 1f;
        if (objectX > mirrorHysteresis)
            return -1f;
        return mirror >= 0f ? 1f : -1f; // close to the center: keep the current side
    }

    void ApplyLayout()
    {
        float scale = LayoutScale();
        for (int i = 0; i < buttons.Count; i++)
        {
            buttons[i].anchoredPosition = LayoutPosition(i);
            buttons[i].localScale = Vector3.one * scale;
        }
    }

    // ---------- follow ----------

    void Update()
    {
        if (!isOpen)
            return;
        if (head == null)
        {
            if (Camera.main == null)
                return;
            head = Camera.main.transform;
        }

        yaw = Mathf.Lerp(yaw, TargetYaw(), 1f - Mathf.Exp(-followSpeed * Time.deltaTime));
        PlaceMenu();
        if (isClosing)
            return;

        UpdateConnector();
        mirror = Mathf.MoveTowards(mirror, MirrorTarget(), Time.deltaTime * 2f / Mathf.Max(0.01f, mirrorDuration));
        if (layoutLive)
            ApplyLayout();
    }

    // Where the menu should be around the object, as a yaw angle (degrees) from the player->object direction.
    float TargetYaw()
    {
        Vector3 toObject = Flat(target.center - head.position);
        float flatDistance = toObject.magnitude;
        if (flatDistance < 1e-4f)
            return yaw;

        float gazeYaw = Vector3.SignedAngle(toObject, Flat(head.forward), Vector3.up); // + = looking right of the object

        // Keep clear of the object: its half width plus the menu's half width, seen from the player.
        float menuHalfWidth = (buttonRadius + 50f) * transform.lossyScale.x;
        float minAngle = Mathf.Max(minSideAngle, Mathf.Atan2(target.halfWidth + menuHalfWidth, flatDistance) * Mathf.Rad2Deg);
        float maxAngle = Mathf.Max(maxFollowAngle, minAngle);

        // The menu sits on the side the player looks toward; looking at the object itself keeps the current side.
        if (gazeYaw > minAngle)
            side = 1;
        else if (gazeYaw < -minAngle)
            side = -1;

        float result = Mathf.Clamp(gazeYaw, -maxAngle, maxAngle);
        if (Mathf.Abs(result) < minAngle)
            result = side * minAngle;
        return result;
    }

    void PlaceMenu()
    {
        Vector3 toObject = Flat(target.center - head.position);
        float flatDistance = toObject.magnitude;
        if (flatDistance < 1e-4f)
            return;

        Vector3 direction = Quaternion.AngleAxis(yaw, Vector3.up) * (toObject / flatDistance);
        float distance = Mathf.Clamp(flatDistance - inFrontOfObject, minDistance, maxDistance);
        Vector3 position = head.position + direction * distance;
        position.y = target.center.y + heightOffset;

        transform.position = position;
        transform.rotation = Quaternion.LookRotation(position - head.position, Vector3.up); // face the player
    }

    static Vector3 Flat(Vector3 v) => new Vector3(v.x, 0f, v.z);

    // ---------- connector line ----------

    void SetUpLines()
    {
        if (s_LineMaterial == null)
            s_LineMaterial = new Material(Shader.Find("Sprites/Default"));

        curve = new Vector3[curveSegments + 1];
        SetUpLine(connector, curve.Length, false);
        SetUpLine(anchorDot, DotSegments, true);
    }

    void SetUpLine(LineRenderer line, int points, bool loop)
    {
        line.useWorldSpace = true;
        line.loop = loop;
        line.positionCount = points;
        line.widthMultiplier = lineWidth;
        line.numCapVertices = 4;
        line.sharedMaterial = s_LineMaterial;
        line.startColor = target.color;
        line.endColor = target.color;
    }

    // Curve from the edge of the center circle (on the side facing the object) to the nearest outline point.
    void UpdateConnector()
    {
        Vector3 hubCenter = hub.position;
        Vector3 anchor = NearestOutlinePoint(hubCenter);
        lastAnchor = anchor;

        Vector3 direction = Vector3.ProjectOnPlane(anchor - hubCenter, transform.forward);
        if (direction.sqrMagnitude < 1e-8f)
            direction = -transform.right;
        direction.Normalize();

        Vector3 start = hubCenter + direction * (hubRadius * hub.lossyScale.x);
        float length = Vector3.Distance(start, anchor);
        Vector3 control1 = start + direction * (length * 0.45f);                                       // leave the menu sideways
        Vector3 control2 = anchor + ((start - anchor).normalized * 0.2f + Vector3.up * 0.3f) * length; // arrive from above

        for (int i = 0; i <= curveSegments; i++)
            curve[i] = Bezier(start, control1, control2, anchor, i / (float)curveSegments);
        connector.SetPositions(curve);

        // Small ring at the anchor, facing the player.
        Vector3 toHead = (head.position - anchor).normalized;
        Vector3 right = Vector3.Cross(Vector3.up, toHead).normalized;
        Vector3 up = Vector3.Cross(toHead, right);
        for (int i = 0; i < DotSegments; i++)
        {
            float a = i * Mathf.PI * 2f / DotSegments;
            dot[i] = anchor + (right * Mathf.Cos(a) + up * Mathf.Sin(a)) * dotRadius;
        }
        anchorDot.SetPositions(dot);
    }

    Vector3 NearestOutlinePoint(Vector3 from)
    {
        if (target.outline == null || target.outline.Length == 0)
            return target.center;
        Vector3 best = target.outline[0];
        float bestDistance = float.MaxValue;
        foreach (var p in target.outline)
        {
            float d = (p - from).sqrMagnitude;
            if (d < bestDistance)
            {
                bestDistance = d;
                best = p;
            }
        }
        return best;
    }

    static Vector3 Bezier(Vector3 a, Vector3 b, Vector3 c, Vector3 d, float t)
    {
        float u = 1f - t;
        return u * u * u * a + 3f * u * u * t * b + 3f * u * t * t * c + t * t * t * d;
    }

    // ---------- animation ----------

    // The center pops in, then the buttons fly out from the center one after another.
    IEnumerator OpenAnimation()
    {
        hub.localScale = Vector3.zero;
        for (int i = 0; i < buttons.Count; i++)
        {
            buttons[i].anchoredPosition = Vector2.zero;
            buttons[i].localScale = Vector3.zero;
        }

        float total = duration + staggerDelay * buttons.Count;
        float elapsed = 0f;
        while (elapsed < total)
        {
            elapsed += Time.deltaTime;
            hub.localScale = Vector3.one * EaseOutBack(Mathf.Clamp01(elapsed / duration));
            float layoutScale = LayoutScale();
            for (int i = 0; i < buttons.Count; i++)
            {
                float e = EaseOutBack(Mathf.Clamp01((elapsed - i * staggerDelay) / duration));
                buttons[i].anchoredPosition = Vector2.LerpUnclamped(Vector2.zero, LayoutPosition(i), e);
                buttons[i].localScale = Vector3.one * (e * layoutScale);
            }
            float lineT = Mathf.Clamp01(elapsed / duration);
            connector.widthMultiplier = lineWidth * lineT;
            anchorDot.widthMultiplier = lineWidth * lineT;
            yield return null;
        }

        hub.localScale = Vector3.one;
        ApplyLayout();
        layoutLive = true;
        connector.widthMultiplier = lineWidth;
        anchorDot.widthMultiplier = lineWidth;
    }

    // The line disappears, the buttons fold back into the center (last one first), then the center shrinks away.
    IEnumerator CloseAnimation()
    {
        connector.enabled = false;
        anchorDot.enabled = false;

        int n = buttons.Count;
        var startPositions = new Vector2[n];
        var startScales = new Vector3[n];
        for (int i = 0; i < n; i++)
        {
            startPositions[i] = buttons[i].anchoredPosition;
            startScales[i] = buttons[i].localScale;
        }
        Vector3 hubStart = hub.localScale;

        float hubDelay = staggerDelay * Mathf.Max(0, n - 1);
        float total = hubDelay + duration;
        float elapsed = 0f;
        while (elapsed < total)
        {
            elapsed += Time.deltaTime;
            for (int i = 0; i < n; i++)
            {
                float e = EaseInBack(Mathf.Clamp01((elapsed - (n - 1 - i) * staggerDelay) / duration));
                buttons[i].anchoredPosition = Vector2.LerpUnclamped(startPositions[i], Vector2.zero, e);
                buttons[i].localScale = Vector3.LerpUnclamped(startScales[i], Vector3.zero, e);
            }
            float hubE = EaseInBack(Mathf.Clamp01((elapsed - hubDelay) / duration));
            hub.localScale = Vector3.LerpUnclamped(hubStart, Vector3.zero, hubE);
            yield return null;
        }
        Destroy(gameObject);
    }

    static float EaseOutBack(float x)
    {
        const float c1 = 1.70158f;
        const float c3 = c1 + 1f;
        return 1f + c3 * Mathf.Pow(x - 1f, 3f) + c1 * Mathf.Pow(x - 1f, 2f);
    }

    static float EaseInBack(float x)
    {
        const float c1 = 1.70158f;
        const float c3 = c1 + 1f;
        return c3 * x * x * x - c1 * x * x;
    }
}

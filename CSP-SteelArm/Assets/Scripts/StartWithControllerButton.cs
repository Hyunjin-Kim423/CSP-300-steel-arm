using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.UI;
using UnityEngine.XR.Templates.MR;

// Starts the app with a controller button instead of the CoachingUI "Start" button.
// Does exactly what that button's OnClick did:
//   GoalManager.CloseTooltipOnly()        closes the coaching UI and turns on passthrough
//   YoloBoxVisualizer.SubscribeROS()      starts receiving YOLO results
//
// It creates itself when the scene has none, so nothing has to be added to the scene.
// To change the button or the hint in the Inspector, add this component to any GameObject in the scene.
public class StartWithControllerButton : MonoBehaviour
{
    [Tooltip("Input System bindings that start the app. Default: B (right) / Y (left).")]
    public string[] bindings =
    {
        "<XRController>{RightHand}/{SecondaryButton}",
        "<XRController>{LeftHand}/{SecondaryButton}"
    };
    [Tooltip("Text shown on the CoachingUI Start button, which becomes a hint instead of a button.")]
    public string startHint = "Press B or Y";
    [Tooltip("Keep the CoachingUI Start button clickable as well (its label is left as it is).")]
    public bool keepStartButton = false;

    InputAction startAction;
    GoalManager goalManager;
    bool started;

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void CreateIfMissing()
    {
        if (FindAnyObjectByType<StartWithControllerButton>() == null)
            new GameObject(nameof(StartWithControllerButton)).AddComponent<StartWithControllerButton>();
    }

    void Start()
    {
        startAction = new InputAction(type: InputActionType.Button);
        foreach (var binding in bindings)
            startAction.AddBinding(binding);
        startAction.Enable();

        goalManager = FindAnyObjectByType<GoalManager>();
        if (goalManager == null)
        {
            Debug.LogWarning("[steelarm] No GoalManager in the scene; the start button will only start the YOLO results.");
            return;
        }
        SetUpStartButton();
    }

    // The CoachingUI step button (GoalManager.m_StepButtonTextField is its label).
    void SetUpStartButton()
    {
        var label = goalManager.m_StepButtonTextField;
        Button button = label != null ? label.GetComponentInParent<Button>() : null;
        if (button == null)
            return;

        if (keepStartButton)
        {
            button.onClick.AddListener(() => started = true); // clicking it also counts as started
            return;
        }
        label.text = startHint;
        button.interactable = false;
    }

    void Update()
    {
        if (!started && startAction.WasPressedThisFrame())
            Begin();
    }

    void Begin()
    {
        started = true;
        startAction.Disable();

        if (goalManager != null)
            goalManager.CloseTooltipOnly();
        if (YoloBoxVisualizer.Instance != null)
            YoloBoxVisualizer.Instance.SubscribeROS();
        else
            Debug.LogWarning("[steelarm] No YoloBoxVisualizer in the scene; YOLO results are not subscribed.");

        Debug.Log("[steelarm] Started with the controller button.");
    }

    void OnDestroy()
    {
        startAction?.Dispose();
    }
}

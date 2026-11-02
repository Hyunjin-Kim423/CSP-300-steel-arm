using System;
using System.Collections.Generic;
using Unity.XR.CoreUtils;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.XR.Interaction.Toolkit.Interactors;
using UnityEngine.XR.Interaction.Toolkit.UI;

// One hand's controller as used by the YOLO visualizer:
//   trigger           -> select the pointed object (opens its menu)
//   primary (A / X)   -> scan
//   aim ray           -> what the controller points at
//   IsPointingAtUI()  -> the hand's XRI ray is on a UI element (e.g. the object menu), so it should not pick objects
public class ControllerInput : IDisposable
{
    public readonly InteractorHandedness handedness;

    readonly InputAction trigger;
    readonly InputAction scanButton;
    readonly InputAction position;
    readonly InputAction rotation;
    readonly List<XRBaseInteractor> uiInteractors = new List<XRBaseInteractor>();

    // hand: "RightHand" or "LeftHand" (Input System usage)
    public ControllerInput(string hand, InteractorHandedness handedness)
    {
        this.handedness = handedness;
        trigger = new InputAction(type: InputActionType.Button, binding: $"<XRController>{{{hand}}}/{{TriggerButton}}");
        scanButton = new InputAction(type: InputActionType.Button, binding: $"<XRController>{{{hand}}}/{{PrimaryButton}}");
        position = new InputAction(binding: $"<XRController>{{{hand}}}/pointerPosition");
        rotation = new InputAction(binding: $"<XRController>{{{hand}}}/pointerRotation");
        trigger.Enable();
        scanButton.Enable();
        position.Enable();
        rotation.Enable();
        FindUIInteractors();
    }

    public bool SelectPressed => trigger.WasPressedThisFrame();
    public bool ScanPressed => scanButton.WasPressedThisFrame();

    public Ray AimRay(XROrigin origin)
    {
        // Controller poses are reported in tracking space, relative to the XR Origin's camera offset.
        Transform trackingSpace = origin.CameraFloorOffsetObject.transform;
        Vector3 worldPosition = trackingSpace.TransformPoint(position.ReadValue<Vector3>());
        Quaternion worldRotation = trackingSpace.rotation * rotation.ReadValue<Quaternion>();
        return new Ray(worldPosition, worldRotation * Vector3.forward);
    }

    // True when an active XRI interactor of this hand currently has its ray on a UI element.
    public bool IsPointingAtUI()
    {
        foreach (var interactor in uiInteractors)
        {
            if (interactor == null || !interactor.isActiveAndEnabled)
                continue;
            if (((IUIInteractor)interactor).TryGetUIModel(out TrackedDeviceModel model) && model.currentRaycast.isValid)
                return true;
        }
        return false;
    }

    // The XRI interactors of this hand that can point at UI (Near-Far / Ray interactors on the controller and hand).
    void FindUIInteractors()
    {
        foreach (var interactor in UnityEngine.Object.FindObjectsByType<XRBaseInteractor>(FindObjectsInactive.Include, FindObjectsSortMode.None))
        {
            if (interactor is IUIInteractor && interactor.handedness == handedness)
                uiInteractors.Add(interactor);
        }
        if (uiInteractors.Count == 0)
            Debug.LogWarning($"[steelarm] No XRI UI interactor found for the {handedness} hand; menu clicks may also select objects behind the menu.");
    }

    public void Dispose()
    {
        trigger.Dispose();
        scanButton.Dispose();
        position.Dispose();
        rotation.Dispose();
    }
}

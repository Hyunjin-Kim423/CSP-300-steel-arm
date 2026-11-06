using UnityEngine;
using UnityEngine.InputSystem;

// Opens / closes the settings panel with the left controller's menu (≡) button (M key in the editor).
// Creates itself when the scene has none, and creates the panel from SteelArmUIAssets on first use.
public class SettingsMenuOpener : MonoBehaviour
{
    InputAction toggle;
    SettingsMenu menu;
    bool warned;

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void CreateIfMissing()
    {
        if (FindAnyObjectByType<SettingsMenuOpener>() == null)
            new GameObject(nameof(SettingsMenuOpener)).AddComponent<SettingsMenuOpener>();
    }

    void Awake()
    {
        toggle = new InputAction(type: InputActionType.Button, binding: "<XRController>{LeftHand}/{MenuButton}");
#if UNITY_EDITOR
        toggle.AddBinding("<Keyboard>/m");
#endif
        toggle.Enable();
    }

    void Update()
    {
        if (toggle.WasPressedThisFrame())
            Toggle();
    }

    public void Toggle()
    {
        if (menu == null && !CreateMenu())
            return;
        menu.Toggle();
    }

    bool CreateMenu()
    {
        SteelArmUIAssets assets = SteelArmUIAssets.Instance;
        if (assets == null || assets.settingsMenuPrefab == null)
        {
            if (!warned)
            {
                warned = true;
                Debug.LogError("[steelarm] Settings menu prefab not found. Run SteelArm > Build Settings Menu Prefab.");
            }
            return false;
        }
        GameObject go = Instantiate(assets.settingsMenuPrefab);
        go.SetActive(false);
        menu = go.GetComponent<SettingsMenu>();
        return menu != null;
    }

    void OnDestroy() => toggle?.Dispose();
}

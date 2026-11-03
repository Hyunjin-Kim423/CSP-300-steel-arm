using UnityEngine;

// Opens the radial object menu for the selected object. Only one menu is open at a time:
// opening a new one plays the closing animation of the previous one.
public class ObjectMenuGenerator : MonoBehaviour
{
    public static ObjectMenuGenerator _instance;

    [Tooltip("Assets/Prefabs/RadialMenu.prefab (assigned by SteelArm > Build Radial Menu Prefabs).")]
    public GameObject _uiPrefab;

    RadialMenu current;

    void Awake()
    {
        if (_instance == null)
            _instance = this;
        else
            Destroy(gameObject);
    }

    public void Open(MenuTarget target)
    {
        if (_uiPrefab == null)
        {
            Debug.LogError("[steelarm] ObjectMenuGenerator has no menu prefab. Run SteelArm > Build Radial Menu Prefabs.");
            return;
        }

        if (current != null)
            current.Close(); // destroys itself after its closing animation

        // Not parented: the menu positions itself in world space, and a parent's scale would change its size.
        GameObject menu = Instantiate(_uiPrefab);
        current = menu.GetComponent<RadialMenu>();
        if (current == null)
        {
            Debug.LogError("[steelarm] The menu prefab has no RadialMenu component. Run SteelArm > Build Radial Menu Prefabs.");
            Destroy(menu);
            return;
        }
        current.Open(target);
    }
}

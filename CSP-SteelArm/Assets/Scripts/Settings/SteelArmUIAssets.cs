using UnityEngine;

// Prefabs that scripts create at runtime without a scene reference.
// Lives at Assets/Resources/SteelArmUIAssets.asset (made by SteelArm > Build Settings Menu Prefab).
public class SteelArmUIAssets : ScriptableObject
{
    public GameObject settingsMenuPrefab;

    static SteelArmUIAssets s_Instance;

    public static SteelArmUIAssets Instance
    {
        get
        {
            if (s_Instance == null)
                s_Instance = Resources.Load<SteelArmUIAssets>("SteelArmUIAssets");
            return s_Instance;
        }
    }
}

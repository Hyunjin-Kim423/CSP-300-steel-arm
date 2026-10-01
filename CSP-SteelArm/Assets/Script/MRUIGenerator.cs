using UnityEngine;

public class MRUIGenerator : MonoBehaviour
{   
    public static MRUIGenerator _instance;

    public GameObject _uiPrefab;
    GameObject currentUI;

    float easing = 5;

    public float maxDistance = 1.0f;
    
    float currentScale;

    void Awake()
    {
        if (_instance == null)
        {
            _instance = this;
        }
        else
        {
            Destroy(gameObject);
        }

        currentScale = _uiPrefab.transform.localScale.x;
    }

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void GenerateUI(Ray ray, float distance, string targetName)
    {
        if (_uiPrefab != null)
        {
            if (currentUI != null)
            {
                Destroy(currentUI);
            }

            float dist = Mathf.Min(distance, maxDistance);
            Vector3 position = ray.GetPoint(dist);
            currentUI = Instantiate(_uiPrefab, position, Quaternion.identity);
            currentUI.transform.position = Vector3.Lerp(currentUI.transform.position, position + new Vector3(0, 0.3f, 0), easing * Time.deltaTime);
            currentUI.transform.localScale = Vector3.one * Mathf.Lerp(currentScale / 2.5f, currentScale * 1.5f, dist / maxDistance);
            currentUI.GetComponent<TestUI>().textMesh.text = targetName;
            currentUI.GetComponent<TestUI>().fixedPosition = position + new Vector3(0, 0.3f, 0);
        }
    }
}

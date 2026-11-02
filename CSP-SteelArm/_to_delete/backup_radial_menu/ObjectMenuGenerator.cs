using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ObjectMenuGenerator : MonoBehaviour
{   
    public static ObjectMenuGenerator _instance;

    public GameObject _uiPrefab;
    GameObject _currentUI;

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

    public void GenerateUI(Ray ray, float distance, RosMessageTypes.Yolo.DetectionMsg data) // change to object data
    {
        if (_uiPrefab != null)
        {
            if (_currentUI != null)
            {
                // 기존 메뉴는 닫힘 애니메이션을 재생한 뒤 스스로 Destroy됨
                _currentUI.GetComponent<ObjectMenu>().Close();
            }

            float dist = Mathf.Min(distance, maxDistance);
            Vector3 _position = ray.GetPoint(dist);
            _currentUI = Instantiate(_uiPrefab, transform);
            _currentUI.transform.position = _position;
            _currentUI.transform.localScale = Vector3.one * Mathf.Lerp(currentScale / 2.5f, currentScale * 1.5f, dist / maxDistance);

            _currentUI.GetComponent<ObjectMenu>().fixedPosition = _position + Vector3.up * 0.15f;
            _currentUI.GetComponent<ObjectMenu>().Open(data);
        }
    }
}

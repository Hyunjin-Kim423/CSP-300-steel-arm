using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit.UI;

public class XRInteractionReceiver : MonoBehaviour
{
    public GameObject _uiPrefab;
    GameObject _currentUIObject;


    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void OnXRControllerPressed() 
    {
        if (_currentUIObject == null)
        {
            _currentUIObject = Instantiate(_uiPrefab, transform);
            _currentUIObject.transform.localPosition = new Vector3(0, 0, 0);
        }
    }
}

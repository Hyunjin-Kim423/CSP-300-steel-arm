using UnityEngine;

public class TestUI : MonoBehaviour
{
    Transform mainCam;
    const float radius = .3f;
    const float orbitSpeed = 10f;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        if (Camera.main != null)
        {
            mainCam = Camera.main.transform;
        }
    }

    // Update is called once per frame
    void Update()
    {
        if (mainCam != null)
        {
            Vector3 newPos = transform.parent.position + (mainCam.position - transform.parent.position).normalized * radius;
            transform.position = Vector3.Lerp(transform.position, newPos, Time.deltaTime * orbitSpeed);

            Vector3 lookDirection = mainCam.position - transform.position;
            transform.rotation = Quaternion.LookRotation(-lookDirection, Vector3.up);
        }
    }

    public void CloseUI()
    {
        Destroy(gameObject);
    }
}

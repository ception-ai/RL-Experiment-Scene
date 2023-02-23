using UnityEngine;

public class CameraExtenededSettings : MonoBehaviour
{
    public float fps = 20;
    private float elapsed;
    private Camera cam;

    void Start()
    {
        cam = GetComponentInChildren<Camera>();
        cam.enabled = false;
    }

    void FixedUpdate()
    {
        
        elapsed += Time.deltaTime;
        if (elapsed > 1 / fps)
        {
            elapsed = 0;
            cam.Render();
        }
    }
    
}

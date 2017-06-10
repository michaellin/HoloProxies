using UnityEngine;
using System.Collections;

public class CameraController : MonoBehaviour {

    public float horizontalSpeed = 2.0F;
    public float verticalSpeed = 2.0F;
    void Update()
    {
        float h = horizontalSpeed * Input.GetAxis("Mouse X");
        float v = -verticalSpeed * Input.GetAxis("Mouse Y");
        transform.RotateAround(transform.position, Vector3.up, h);
        transform.Rotate(v, 0, 0);
    }
}

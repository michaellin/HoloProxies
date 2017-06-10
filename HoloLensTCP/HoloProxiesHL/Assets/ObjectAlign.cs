using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.VR.WSA.Input;

public class ObjectAlign : MonoBehaviour {

    private bool anchored = false;
    private bool needUpdate = false;
    private Matrix4x4 poseMat;

    public GameObject obj;
    public GameObject obj2;
    private Transform objTransform;
    private Transform objTransform2;

    bool usingObj;

    private GestureRecognizer gestureRecognizer;

    // Use this for initialization
    void Start () {
        objTransform = obj.GetComponent<Transform>();
        objTransform2 = obj2.GetComponent<Transform>();
        obj.SetActive( true );
        obj2.SetActive( false );
        usingObj = true;
        //gestureRecognizer = new GestureRecognizer();
        //gestureRecognizer.SetRecognizableGestures( GestureSettings.Tap );

        //gestureRecognizer.TappedEvent += anchorObject;
        //gestureRecognizer.StartCapturingGestures();
    }
	
	// Update is called once per frame
	void FixedUpdate () {
        if (Input.GetKeyDown(KeyCode.A))
        {
            anchored ^= true;
        }
        else if (Input.GetKeyDown(KeyCode.UpArrow))
        {
            transform.position += Vector3.up* 0.002f;
        }
        else if (Input.GetKeyDown( KeyCode.DownArrow ))
        {
            transform.position -= Vector3.up * 0.002f;
        }
        else if (Input.GetKeyDown( KeyCode.RightArrow ))
        {
            transform.position += (Camera.main.transform.TransformPoint( Vector3.right * 0.002f ) - Camera.main.transform.position);
        }
        else if (Input.GetKeyDown( KeyCode.LeftArrow ))
        {
            transform.position -= (Camera.main.transform.TransformPoint( Vector3.right * 0.002f ) - Camera.main.transform.position);
        }
        else if (Input.GetKeyDown( KeyCode.Alpha1 ))
        {
            obj.SetActive( true );
            obj2.SetActive( false );
            usingObj = true;
        }
        else if (Input.GetKeyDown( KeyCode.Alpha2 ))
        {
            obj.SetActive( false );
            obj2.SetActive( true );
            usingObj = false;
        }


        if (!anchored)
        {
            transform.position = Camera.main.transform.position + Camera.main.transform.forward * 0.6f;
            transform.rotation = Quaternion.AngleAxis( Camera.main.transform.rotation.eulerAngles.y + 180.0f, Vector3.up );
        }

        if (needUpdate)
        {
            Matrix4x4 newMat = RHSToLHSConversion( poseMat );
            Vector3 relPos = ExtractPosition( newMat );
            Quaternion relRot = ExtractRotation( newMat );
            objTransform.rotation = transform.rotation * relRot;
            objTransform.position = transform.TransformPoint( relPos );
            objTransform2.rotation = transform.rotation * relRot;
            objTransform2.position = transform.TransformPoint( relPos );
            if (usingObj)
            {
                objTransform.position = transform.TransformPoint( relPos ) + Vector3.up * 0.1f;
            }
            needUpdate = false;
        }
        
	}

    public void updatePoseMatrix( Matrix4x4 mat )
    {
        //Debug.Log( "update" );
        if (!needUpdate)
        {
            poseMat = mat;
            needUpdate = true;
        }
    }

    private static Matrix4x4 RHSToLHSConversion( Matrix4x4 matrix )
    {
        Vector3 pos = ExtractPosition( matrix );
        pos.y *= -1;
        pos.x *= -1;
        Quaternion q = GetQuaternionFromMatrix( matrix );
        Vector3 eulers = q.eulerAngles;
        eulers.y *= -1;
        eulers.x *= -1;

        Quaternion qx = Quaternion.AngleAxis( eulers.x, Vector3.right );
        Quaternion qy = Quaternion.AngleAxis( eulers.y, Vector3.up );
        Quaternion qz = Quaternion.AngleAxis( eulers.z, Vector3.forward );
        Quaternion qq = qy * qx * qz;
        return Matrix4x4.TRS( pos, qq, Vector3.one );

    }

   // static function MayaRotationToUnity( rotation : Vector3) : Quaternion {
   //var flippedRotation : Vector3 = Vector3( rotation.x, -rotation.y, -rotation.z); // flip Y and Z axis for right->left handed conversion
   //                                                                                // convert XYZ to ZYX
   // var qx : Quaternion = Quaternion.AngleAxis(flippedRotation.x, Vector3.right);
   //var qy : Quaternion = Quaternion.AngleAxis(flippedRotation.y, Vector3.up);
   //var qz : Quaternion = Quaternion.AngleAxis(flippedRotation.z, Vector3.forward);
   //var qq : Quaternion = qz* qy * qx ; // this is the order
   //return qq;


private static Quaternion GetQuaternionFromMatrix( Matrix4x4 m )
{
    Quaternion q = new Quaternion();
    q.w = Mathf.Sqrt( Mathf.Max( 0, 1 + m[0, 0] + m[1, 1] + m[2, 2] ) ) / 2;
    q.x = Mathf.Sqrt( Mathf.Max( 0, 1 + m[0, 0] - m[1, 1] - m[2, 2] ) ) / 2;
    q.y = Mathf.Sqrt( Mathf.Max( 0, 1 - m[0, 0] + m[1, 1] - m[2, 2] ) ) / 2;
    q.z = Mathf.Sqrt( Mathf.Max( 0, 1 - m[0, 0] - m[1, 1] + m[2, 2] ) ) / 2;
    q.x *= Mathf.Sign( q.x * (m[2, 1] - m[1, 2]) );
    q.y *= Mathf.Sign( q.y * (m[0, 2] - m[2, 0]) );
    q.z *= Mathf.Sign( q.z * (m[1, 0] - m[0, 1]) );
    return q;
}


private static Quaternion ExtractRotation( Matrix4x4 matrix )
    {
        Vector3 forward;
        forward.x = matrix.m02;
        forward.y = matrix.m12;
        forward.z = matrix.m22;

        Vector3 upwards;
        upwards.x = matrix.m01;
        upwards.y = matrix.m11;
        upwards.z = matrix.m21;

        return Quaternion.LookRotation( forward, upwards );
    }

    private static Vector3 ExtractPosition( Matrix4x4 matrix )
    {
        Vector3 position;
        position.x = matrix.m03;
        position.y = matrix.m13;
        position.z = matrix.m23;
        return position;
    }


    void anchorObject(InteractionSourceKind source, int tapCount, Ray headRay)
    {
        anchored = true;
    }

    void OnDestroy()
    {
        gestureRecognizer.StopCapturingGestures();
        gestureRecognizer.TappedEvent -= anchorObject;
    }
}

using UnityEngine;

public class CameraFollow : MonoBehaviour
{
    [SerializeField] Transform target;
    public Vector3 offset = new Vector3(0f, 3.5f, -6f);
    public Vector3 povOffset = new Vector3(0f, 1.5f, 0f);
    public Vector3 regularOffset = new Vector3(0, 2, 4);

    // Damping in 1/seconds (higher = snappier). Works consistently across frame rates.
    [SerializeField, Min(0f)] float positionDamping = 8f;
    [SerializeField, Min(0f)] float rotationDamping = 10f;

    // Optional tiny dead-zone to ignore micro jitter (in meters / degrees)
    [SerializeField, Min(0f)] float positionDeadZone = 0.01f;
    [SerializeField, Min(0f)] float rotationDeadZoneDeg = 0.2f;

    public bool freeRotation = true;
    public bool inPOV = false;

    // ▶ Added:
    [Header("Pre-start creep")]
    public bool gameStarted = false;                 // Set this true when your game starts
    [SerializeField, Min(0f)] float startApproachSpeed = 0.5f; // m/s while game hasn't started

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.V))
        {
            freeRotation = !freeRotation;
            if (freeRotation)
            {
                offset = regularOffset;
                inPOV = false;
            }
            else
            {
                offset = povOffset;
                inPOV = true;
            }
        }
    }

    void LateUpdate()
    {
        if (!target) return;

        // desired position: offset in target local space
        Vector3 desiredPos = target.TransformPoint(offset);

        if (!gameStarted)
        {
            // ▶ Before game starts: move very slowly toward the target position (frame-rate independent).
            transform.position = Vector3.MoveTowards(
                transform.position,
                desiredPos,
                startApproachSpeed * Time.deltaTime
            );
        }
        else
        {
            // ▶ After game starts: your normal smoothed follow with dead-zone
            Vector3 delta = desiredPos - transform.position;
            if (delta.sqrMagnitude > positionDeadZone * positionDeadZone || inPOV)
            {
                float t = 1f - Mathf.Exp(-positionDamping * Time.deltaTime);
                transform.position = Vector3.Lerp(transform.position, desiredPos, t);
            }
        }

        // desired rotation: look at target (unchanged)
        Quaternion desiredRot = Quaternion.LookRotation(target.position - transform.position, Vector3.up);
        if (!freeRotation)
        {
            desiredRot = Quaternion.Euler(-target.eulerAngles);
        }

        float ang = Quaternion.Angle(transform.rotation, desiredRot);
        if (ang > rotationDeadZoneDeg || inPOV)
        {
            float tr = 1f - Mathf.Exp(-rotationDamping * Time.deltaTime);
            transform.rotation = Quaternion.Slerp(transform.rotation, desiredRot, tr);
        }
    }
}

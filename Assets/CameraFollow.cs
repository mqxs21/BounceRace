using UnityEngine;

public class CameraFollow : MonoBehaviour
{
    [SerializeField] Transform target;
    [SerializeField] Vector3 offset = new Vector3(0f, 3.5f, -6f);

    // Damping in 1/seconds (higher = snappier). Works consistently across frame rates.
    [SerializeField, Min(0f)] float positionDamping = 8f;
    [SerializeField, Min(0f)] float rotationDamping = 10f;

    // Optional tiny dead-zone to ignore micro jitter (in meters / degrees)
    [SerializeField, Min(0f)] float positionDeadZone = 0.01f;
    [SerializeField, Min(0f)] float rotationDeadZoneDeg = 0.2f;

    void LateUpdate()
    {
        if (!target) return;

        // desired position: offset in target local space
        Vector3 desiredPos = target.TransformPoint(offset);

        // dead-zone
        Vector3 delta = desiredPos - transform.position;
        if (delta.sqrMagnitude > positionDeadZone * positionDeadZone)
        {
            // exponential smoothing (frame-rate independent)
            float t = 1f - Mathf.Exp(-positionDamping * Time.deltaTime);
            transform.position = Vector3.Lerp(transform.position, desiredPos, t);
        }

        // desired rotation: look at target
        Quaternion desiredRot = Quaternion.LookRotation(target.position - transform.position, Vector3.up);

        // dead-zone for rotation
        float ang = Quaternion.Angle(transform.rotation, desiredRot);
        if (ang > rotationDeadZoneDeg)
        {
            float tr = 1f - Mathf.Exp(-rotationDamping * Time.deltaTime);
            transform.rotation = Quaternion.Slerp(transform.rotation, desiredRot, tr);
        }
    }
}

using UnityEngine;

public class CarSpinner : MonoBehaviour
{
    [Header("Rotation Settings")]
    public float minRotationSpeed = 20f;
    public float maxRotationSpeed = 120f;

    [Header("Movement Settings")]
    public Vector3 positionRange = new Vector3(5, 0, 5); // X,Z range car can move
    public float changePositionInterval = 3f; // How often to teleport

    private Vector3 rotationAxis;
    private float rotationSpeed;
    private float timer;

    void Start()
    {
        SetRandomRotation();
        SetRandomPosition();
    }

    void Update()
    {
        // Rotate the car
        transform.Rotate(rotationAxis * rotationSpeed * Time.deltaTime);

        // Timer to change position occasionally
        timer += Time.deltaTime;
        if (timer >= changePositionInterval)
        {
            SetRandomRotation();
           // SetRandomPosition();
            timer = 0f;
        }
    }

    void SetRandomRotation()
    {
        rotationAxis = Random.onUnitSphere; // random 3D direction
        rotationSpeed = Random.Range(minRotationSpeed, maxRotationSpeed);
    }

    void SetRandomPosition()
    {
        transform.position = new Vector3(
            Random.Range(-positionRange.x, positionRange.x),
            Random.Range(-positionRange.y, positionRange.y),
            Random.Range(-positionRange.z, positionRange.z)
        );
    }
}

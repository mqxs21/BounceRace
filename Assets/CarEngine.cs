using UnityEngine;

public class CarEngine : MonoBehaviour
{
    public Rigidbody rb;
    public CarController carController;
    public AudioSource engineSound;
    public

    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        engineSound.pitch = ((int)(CarController.velocity * 3.6)) / 50f;
    }
}

using System;
using UnityEngine;

public class CarEngine : MonoBehaviour
{
    public AudioSource engineSound;
    public CarController carController;
    void Start()
    {
       // carController = GetComponent<CarController>();
    }

    // Update is called once per frame
    void Update()
    {
        float isTurningMultiplier = carController.isTurning ? 0.7f : 1f;
        float targetPitch = ((CarController.velocity * 3.6f) / 80) * UnityEngine.Random.Range(0.9f, 1.1f) * isTurningMultiplier;
        float isInAirMultiplier = carController.isGrounded ? 1f : 0.5f;
        engineSound.pitch = Mathf.Lerp(engineSound.pitch, targetPitch, Time.deltaTime * 10f);

        if (LapProgress.gameIsDone)
        {
            engineSound.enabled = false;
        }
    }
}

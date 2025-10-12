using UnityEngine;

public class WheelSounder : MonoBehaviour
{
    public WheelCollider wheelCollider;
    public AudioSource bounceSound;

    private bool wasGrounded = false;
    public float initalPitch;
    void Start()
    {
        wheelCollider = GetComponent<WheelCollider>();
        bounceSound = GetComponent<AudioSource>();
        initalPitch = bounceSound.pitch;
    }

    void Update()
    {
        bool isGrounded = wheelCollider.GetGroundHit(out WheelHit hit);

        // Just touched the ground
        if (isGrounded && !wasGrounded)
        {
            float impactStrength = Mathf.InverseLerp(0f, 10000f, hit.force); // normalize to 0–1 range
            bounceSound.volume = Mathf.Lerp(0.2f, 1f, impactStrength);
            bounceSound.pitch = initalPitch + UnityEngine.Random.Range(-0.2f,0.2f); // adds variety
            bounceSound.Play();
        }

        // Just left the ground
        if (!isGrounded && wasGrounded)
        {
            float impactStrength = Mathf.InverseLerp(0f, 10000f, hit.force); // normalize to 0–1 range
            bounceSound.volume = Mathf.Lerp(0.2f, 1f, impactStrength);
            bounceSound.pitch = initalPitch + UnityEngine.Random.Range(-0.2f,0.2f);
            bounceSound.Play();
        }

        wasGrounded = isGrounded;
    }
}

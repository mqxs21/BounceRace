using System;
using System.Collections;
using UnityEngine.SceneManagement;
using System.Collections.Generic;
using UnityEngine;

public class CarController : MonoBehaviour
{
    private float horizontalInput;
    private float verticalInput;
    private float steerAngle;
    private bool isBreaking = false;

    public WheelCollider frontLeftWheelCollider;
    public WheelCollider frontRightWheelCollider;
    public WheelCollider rearLeftWheelCollider;
    public WheelCollider rearRightWheelCollider;
    public Transform frontLeftWheelTransform;
    public Transform frontRightWheelTransform;
    public Transform rearLeftWheelTransform;
    public Transform rearRightWheelTransform;

    public float maxSteeringAngle = 30f;
    public float motorForce = 50f;
    public float brakeForce = 0f;

    public Rigidbody carRigidbody;

    public Transform bodyVisual;
    public Vector3 initalBodyRotation;

    public bool isGrounded = false;

    public float lastVert = 0f;

    public Vector3 normalGravity = new Vector3(0f, -20f, 0f);
    public Vector3 leftSideGravity = new Vector3(-20f, 0f, 0f);
    public Vector3 rightSideGravity = new Vector3(20f, 0f, 0f);
    public Vector3 airGravity = new Vector3(0f, -50f, 0f);

    void Start()
    {
        carRigidbody = GetComponent<Rigidbody>();
        initalBodyRotation = bodyVisual.localEulerAngles;
        carRigidbody.centerOfMass += new Vector3(0f, -0.8f, 0f); 
        carRigidbody.maxAngularVelocity = 1.5f;
        Physics.gravity = normalGravity;

    }
    private void FixedUpdate()
    {
        GetInput();
        HandleMotor();
        HandleSteering();
        UpdateWheels();

        if (verticalInput == 0f || isBreaking)
        {
            Debug.Log("Trying to brake");
            frontLeftWheelCollider.motorTorque = 0f;
            frontRightWheelCollider.motorTorque = 0f;
            rearLeftWheelCollider.motorTorque = 0f;
            rearRightWheelCollider.motorTorque = 0f;



            Vector3 currentVelocity = carRigidbody.linearVelocity;

            float newMagnitude = Mathf.Lerp(currentVelocity.magnitude, 0, Time.deltaTime * 4f);
            currentVelocity = currentVelocity.normalized * newMagnitude;
            currentVelocity.y = carRigidbody.linearVelocity.y; // preserve y velocity for jumps/gravity
            carRigidbody.linearVelocity = currentVelocity;
            
            frontLeftWheelCollider.brakeTorque = carRigidbody.linearVelocity.magnitude;
            frontRightWheelCollider.brakeTorque = carRigidbody.linearVelocity.magnitude;
            rearLeftWheelCollider.brakeTorque = carRigidbody.linearVelocity.magnitude;
            rearRightWheelCollider.brakeTorque = carRigidbody.linearVelocity.magnitude;
        }
        if (Mathf.Abs(horizontalInput) >= 0.1f)
        {
            bodyVisual.localEulerAngles = new Vector3(initalBodyRotation.x, initalBodyRotation.y, steerAngle*0.3f);
        }
        else
        {
            bodyVisual.localEulerAngles = initalBodyRotation;
        }


       
    }

    public void CheckGround()
    {
        RaycastHit hit;
        if (Physics.Raycast(transform.position, Vector3.down, out hit, 0.7f))
        {
            isGrounded = true;
        }
        else
        {
            isGrounded = false;
        }
        Debug.DrawRay(transform.position, Vector3.down * 0.7f, Color.red);
    }

    void Update()
    {
        CheckGround();
        if (Input.GetKeyDown(KeyCode.Space) && isGrounded)
        {
            //JUMP!;
            carRigidbody.linearVelocity += new Vector3(0f, 20f, 0f);
        }
        if (Input.GetKey(KeyCode.E))
        {
            isBreaking = true;
        }else
        {
            isBreaking = false;
        }

        if (!isGrounded)
        {
            Physics.gravity = airGravity;
        }else
        {
            Physics.gravity = normalGravity;
        }
        if (Input.GetKey(KeyCode.R))
        {
            SceneManager.LoadScene(SceneManager.GetActiveScene().name);
        }
    }

    private void GetInput()
    {
        horizontalInput = Input.GetAxis("Horizontal");
        verticalInput = -Input.GetAxis("Vertical");
        lastVert = verticalInput;
        // isBreaking = Input.GetKey(KeyCode.Space);
    }

 private float yawBrake = 10f; // tweak 10–40

private void HandleSteering()
{
    steerAngle = maxSteeringAngle * horizontalInput;
    frontLeftWheelCollider.steerAngle  = steerAngle * 0.5f;
    frontRightWheelCollider.steerAngle = steerAngle * 0.5f;

    // When no steer input, gently brake ONLY yaw so car stops turning
    if (Mathf.Abs(horizontalInput) < 0.01f)
    {
        // current angular velocity
        Vector3 w = carRigidbody.angularVelocity;

        // extract yaw component (about world up)
        float yaw = Vector3.Dot(w, Vector3.up);

        // apply opposite torque proportional to yaw (doesn't touch linear speed)
        // ForceMode.Acceleration is mass-independent and smooth.
        carRigidbody.AddTorque(-Vector3.up * yaw * yawBrake, ForceMode.Acceleration);
    }
}




void ApplyAntiRoll(WheelCollider left, WheelCollider right, float stiffness)
{
    float travelL = 1f, travelR = 1f;
    bool groundedL = left.GetGroundHit(out WheelHit hitL);
    bool groundedR = right.GetGroundHit(out WheelHit hitR);

    if (groundedL)
        travelL = (-left.transform.InverseTransformPoint(hitL.point).y - left.radius) / left.suspensionDistance;
    if (groundedR)
        travelR = (-right.transform.InverseTransformPoint(hitR.point).y - right.radius) / right.suspensionDistance;

    float force = (travelL - travelR) * stiffness;

    if (groundedL) carRigidbody.AddForceAtPosition(left.transform.up * -force,  left.transform.position);
    if (groundedR) carRigidbody.AddForceAtPosition(right.transform.up *  force,  right.transform.position);
}


    private void HandleMotor()
    {
        frontLeftWheelCollider.motorTorque = verticalInput * motorForce;
        frontRightWheelCollider.motorTorque = verticalInput * motorForce;
        
        brakeForce = 0f;
        frontLeftWheelCollider.brakeTorque = brakeForce;
        frontRightWheelCollider.brakeTorque = brakeForce;
        rearLeftWheelCollider.brakeTorque = brakeForce;
        rearRightWheelCollider.brakeTorque = brakeForce;
    }

    private void UpdateWheels()
    {
        UpdateWheelPos(frontLeftWheelCollider, frontLeftWheelTransform);
        UpdateWheelPos(frontRightWheelCollider, frontRightWheelTransform);
        UpdateWheelPos(rearLeftWheelCollider, rearLeftWheelTransform);
        UpdateWheelPos(rearRightWheelCollider, rearRightWheelTransform);
    }

    private void UpdateWheelPos(WheelCollider wheelCollider, Transform trans)
    {
        Vector3 pos;
        Quaternion rot;
        wheelCollider.GetWorldPose(out pos, out rot);
        trans.rotation = rot;
        trans.position = pos;
    }

}
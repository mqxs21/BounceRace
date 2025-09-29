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
    public float maxSpeed = 100;

    public Rigidbody carRigidbody;

    public Transform bodyVisual;
    public Vector3 initalBodyRotation;

    public bool isGrounded = false;

    public float lastVert = 0f;

    public Vector3 normalGravity = new Vector3(0f, -20f, 0f);
    public Vector3 leftSideGravity = new Vector3(-20f, 0f, 0f);
    public Vector3 rightSideGravity = new Vector3(20f, 0f, 0f);
    public Vector3 airGravity = new Vector3(0f, -50f, 0f);

    // ===================== DRIFT ADDITIONS (minimal) =====================
    [Header("Drift")]
    public KeyCode driftKey = KeyCode.LeftShift;
    public bool isDrifting = false;

    public float driftSteerMultiplier = 1.6f;       // easier to turn while drifting
    public float driftTorqueMultiplier = 0.35f;     // less engine power while drifting
    public float handbrakeTorque = 1200f;           // rear lock to start slide
    public float driftYawPower = 25f;               // extra yaw torque during drift
    public float driftMaxLongitudinalSpeed = 25f;   // m/s forward cap while drifting
    public float driftLongitudinalDamping = 3f;     // how strongly to resist excess forward speed

    // friction tuning (only stiffness changed while drifting)
    public float rearSidewaysStiffness = 0.6f;      // lower = more slide
    public float rearForwardStiffness  = 0.8f;      // slightly less forward grip
    public float frontSidewaysBoost    = 1.2f;      // a bit more front bite for control

    // cache originals so we can restore after drifting
    private WheelFrictionCurve flFwd0, frFwd0, rlFwd0, rrFwd0;
    private WheelFrictionCurve flSide0, frSide0, rlSide0, rrSide0;
    // =====================================================================

    void Start()
    {
        Application.targetFrameRate = 120;
        carRigidbody = GetComponent<Rigidbody>();
        initalBodyRotation = bodyVisual.localEulerAngles;
        carRigidbody.centerOfMass += new Vector3(-0.3f, -0.8f, 0f); 
        carRigidbody.maxAngularVelocity = 1.5f;
        Physics.gravity = normalGravity;

        // ===== DRIFT: cache original friction curves =====
        flFwd0  = frontLeftWheelCollider.forwardFriction;
        frFwd0  = frontRightWheelCollider.forwardFriction;
        rlFwd0  = rearLeftWheelCollider.forwardFriction;
        rrFwd0  = rearRightWheelCollider.forwardFriction;

        flSide0 = frontLeftWheelCollider.sidewaysFriction;
        frSide0 = frontRightWheelCollider.sidewaysFriction;
        rlSide0 = rearLeftWheelCollider.sidewaysFriction;
        rrSide0 = rearRightWheelCollider.sidewaysFriction;
        // ================================================
    }
    private void FixedUpdate()
    {
        GetInput();
        HandleMotor();
        HandleSteering();
        UpdateWheels();

        // ===== DRIFT: keep your original brake block but don't run it while drifting =====
        if (!isDrifting && (verticalInput == 0f || isBreaking))
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

        // ===== DRIFT: apply drift friction, yaw assist, and forward speed limit =====
        if (isDrifting)
        {
            // soften rear grip, boost front bite
            SetStiffness(rearLeftWheelCollider,  rlFwd0.stiffness  * rearForwardStiffness,  rlSide0.stiffness * rearSidewaysStiffness);
            SetStiffness(rearRightWheelCollider, rrFwd0.stiffness  * rearForwardStiffness,  rrSide0.stiffness * rearSidewaysStiffness);
            SetStiffness(frontLeftWheelCollider, flFwd0.stiffness,                           flSide0.stiffness * frontSidewaysBoost);
            SetStiffness(frontRightWheelCollider,frFwd0.stiffness,                           frSide0.stiffness * frontSidewaysBoost);

            // extra yaw to help sustain drift in steer direction
            carRigidbody.AddTorque(Vector3.up * horizontalInput * driftYawPower, ForceMode.Acceleration);

            // cap only the forward component (so you can steer without rocketing forward)
            Vector3 v = carRigidbody.linearVelocity;
            float vLong = Vector3.Dot(v, transform.forward);
            if (vLong > driftMaxLongitudinalSpeed)
            {
                float excess = vLong - driftMaxLongitudinalSpeed;
                carRigidbody.AddForce(-transform.forward * (excess * driftLongitudinalDamping), ForceMode.Acceleration);
            }
        }
        else
        {
            // restore original grip
            RestoreFriction();
        }
        // ===============================================================================

        if (Mathf.Abs(horizontalInput) >= 0.1f)
        {
            bodyVisual.localEulerAngles = new Vector3(initalBodyRotation.x, initalBodyRotation.y, steerAngle * 0.3f);
        }
        else
        {
            bodyVisual.localEulerAngles = initalBodyRotation;
        }
        if (carRigidbody.linearVelocity.magnitude> maxSpeed)
        {
            carRigidbody.linearVelocity = carRigidbody.linearVelocity.normalized * maxSpeed;
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

        // ===== DRIFT: read drift key (only allow when grounded) =====
        isDrifting = Input.GetKey(driftKey) && isGrounded;
        // ============================================================

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
        // ===== DRIFT: stronger steering while drifting =====
        steerAngle = maxSteeringAngle * horizontalInput * (isDrifting ? driftSteerMultiplier : 1f);
        // ===================================================

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
        // ===== DRIFT: special motor/brake behavior =====
        if (isDrifting)
        {
            frontLeftWheelCollider.motorTorque  = verticalInput * motorForce * driftTorqueMultiplier;
            frontRightWheelCollider.motorTorque = verticalInput * motorForce * driftTorqueMultiplier;

            // rear “handbrake” to keep rear loose
            frontLeftWheelCollider.brakeTorque  = 0f;
            frontRightWheelCollider.brakeTorque = 0f;
            rearLeftWheelCollider.brakeTorque   = handbrakeTorque;
            rearRightWheelCollider.brakeTorque  = handbrakeTorque;
            return;
        }
        // =================================================

        frontLeftWheelCollider.motorTorque = verticalInput * motorForce;
        frontRightWheelCollider.motorTorque = verticalInput * motorForce;

        brakeForce =  verticalInput >= 0 ? 2000f : 0;
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

    // ===== DRIFT: helpers to adjust/restore stiffness only =====
    void SetStiffness(WheelCollider wc, float fwd, float side)
    {
        var f = wc.forwardFriction;  f.stiffness = fwd;  wc.forwardFriction = f;
        var s = wc.sidewaysFriction; s.stiffness = side; wc.sidewaysFriction = s;
    }

    void RestoreFriction()
    {
        frontLeftWheelCollider.forwardFriction  = flFwd0;
        frontRightWheelCollider.forwardFriction = frFwd0;
        rearLeftWheelCollider.forwardFriction   = rlFwd0;
        rearRightWheelCollider.forwardFriction  = rrFwd0;

        frontLeftWheelCollider.sidewaysFriction  = flSide0;
        frontRightWheelCollider.sidewaysFriction = frSide0;
        rearLeftWheelCollider.sidewaysFriction   = rlSide0;
        rearRightWheelCollider.sidewaysFriction  = rrSide0;
    }
    // ===========================================================
}

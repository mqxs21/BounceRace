using System;
using System.Collections;
using UnityEngine.SceneManagement;
using System.Collections.Generic;
using UnityEngine;
using TMPro;
using UnityEngine.U2D;
using UnityEditor.Rendering;

public class CarController : MonoBehaviour
{
    public static float velocity;
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
    public float maxSpeedHardCap = 150;

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

    public Vector3 normalCOM;
    public Vector3 airCOM;

    public float driftElapsedTime = 0f;
    public float timeToStartDriftEffect = 0.5f;

    public ParticleSystem leftDriftEffect;
    public ParticleSystem rightDriftEffect;
    public ParticleSystem boostEffect;
    public ParticleSystem windEffect;
    public ParticleSystem vfxBoostEffect;

    public float initYStartDrift = 0f;
    public bool stopAngleChange = false;

    // ===== NEW: Drift yaw clamp =====
    [SerializeField] float maxDriftYaw = 60f;   // degrees allowed left/right from start
    private float driftStartYaw;                // yaw (degrees) at drift start
    // =================================
    public float maxDriftTime = 2f;            // seconds

    // ===== NEW: Drift staging (pose -> actual) + end boost =====
    [Header("Drift Staging")]
    public bool isDriftPosing = false;          // true during pre-drift pose
    public float driftEndBoost = 8f;            // forward boost on successful end
    private float driftPoseTimer = 0f;          // counts pose time
    private float driftTotalTimer = 0f;         // counts total from initial press
    private bool driftKeyWasHeld = false;       // input edge detection
    private bool didActualDrift = false;        // whether we reached real drifting this cycle
    private float driftTotalTime = 0f;
    // ===========================================================
    public float lastDriftDir = 0f;

    public float bodyVisualMultiplier = 2f;
    public float bodyVisualMultiplierDrift = 2f;

    public float windMaxRadius = 43f;
    public float windMinRadius = 33f;

    public bool leftBackWheelIsGrounded = false;
    public bool rightBackWheelIsGrounded = false;

    public float stuckYawIfOppositeHeld = 0f;

    public TrailRenderer leftBackWheelTrail;
    public TrailRenderer rightBackWheelTrail;

    // UI ELEMENTS ===========================================================
    public TextMeshProUGUI speedText;
    // ===========================================================
    public LayerMask excludePushLayers;

    public bool stopTurnTorque = false;
    void Start()
    {
        Application.targetFrameRate = 120;
        carRigidbody = GetComponent<Rigidbody>();
        initalBodyRotation = bodyVisual.localEulerAngles;
        carRigidbody.centerOfMass += new Vector3(0f, -0.8f, -0.1f);
        normalCOM = carRigidbody.centerOfMass;
        airCOM = carRigidbody.centerOfMass + new Vector3(0f, 0f, -0.5f);
        carRigidbody.maxAngularVelocity = 1.5f;
        Physics.gravity = normalGravity;

        // ===== DRIFT: cache original friction curves =====
        flFwd0 = frontLeftWheelCollider.forwardFriction;
        frFwd0 = frontRightWheelCollider.forwardFriction;
        rlFwd0 = rearLeftWheelCollider.forwardFriction;
        rrFwd0 = rearRightWheelCollider.forwardFriction;

        flSide0 = frontLeftWheelCollider.sidewaysFriction;
        frSide0 = frontRightWheelCollider.sidewaysFriction;
        rlSide0 = rearLeftWheelCollider.sidewaysFriction;
        rrSide0 = rearRightWheelCollider.sidewaysFriction;
        // ================================================
        leftDriftEffect.Stop();
        rightDriftEffect.Stop();

        TuneFriction(frontLeftWheelCollider, 2.3f, 2.1f);
        TuneFriction(frontRightWheelCollider, 2.3f, 2.1f);
        TuneFriction(rearLeftWheelCollider, 2.2f, 1.9f);
        TuneFriction(rearRightWheelCollider, 2.2f, 1.9f);

        maxSpeed /= 3.6f; //convert from km/h
    }

    private void FixedUpdate()
    {
        GetInput();
//        Debug.Log(carRigidbody.linearVelocity.magnitude);
        
        HandleMotor();
        HandleSteering();
        UpdateWheels();
        velocity = carRigidbody.linearVelocity.magnitude; // km/h

        if (stopAngleChange && !isDriftPosing && !isDrifting)
        {
            //in drift turning
            Vector3 currentRotation = transform.rotation.eulerAngles;
            currentRotation.y = initYStartDrift + (maxSteeringAngle * horizontalInput * 1.5f);
            transform.rotation = Quaternion.Slerp(transform.rotation, Quaternion.Euler(currentRotation), Time.deltaTime * 2f);
        }
        /*  else if (stopAngleChange && isDriftPosing)
          {
              Vector3 currentRotation = transform.rotation.eulerAngles;
              currentRotation.y = initYStartDrift + (maxSteeringAngle * horizontalInput * 1.5f);
              transform.rotation = Quaternion.Slerp(transform.rotation, Quaternion.Euler(currentRotation), Time.deltaTime * 2f);
          }
          else if (stopAngleChange && !isDriftPosing && isDrifting)
          {
              Vector3 currentRotation = transform.rotation.eulerAngles;
              currentRotation.y = initYStartDrift + (maxSteeringAngle * horizontalInput * 1.5f);
              transform.rotation = Quaternion.Slerp(transform.rotation, Quaternion.Euler(currentRotation), Time.deltaTime * 2f);
          }*/
        if (horizontalInput != 0)
        {
            carRigidbody.AddTorque(Vector3.up * horizontalInput * driftYawPower * 2, ForceMode.Force);
        }
        else
        {
            
        }
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
                currentVelocity.y = carRigidbody.linearVelocity.y;
                carRigidbody.linearVelocity = currentVelocity;

                frontLeftWheelCollider.brakeTorque = carRigidbody.linearVelocity.magnitude;
                frontRightWheelCollider.brakeTorque = carRigidbody.linearVelocity.magnitude;
                rearLeftWheelCollider.brakeTorque = carRigidbody.linearVelocity.magnitude;
                rearRightWheelCollider.brakeTorque = carRigidbody.linearVelocity.magnitude;
            }

        if (isDrifting)
        {
            SetStiffness(rearLeftWheelCollider, rlFwd0.stiffness * rearForwardStiffness, rlSide0.stiffness * rearSidewaysStiffness);
            SetStiffness(rearRightWheelCollider, rrFwd0.stiffness * rearForwardStiffness, rrSide0.stiffness * rearSidewaysStiffness);
            SetStiffness(frontLeftWheelCollider, flFwd0.stiffness, flSide0.stiffness * frontSidewaysBoost);
            SetStiffness(frontRightWheelCollider, frFwd0.stiffness, frSide0.stiffness * frontSidewaysBoost);
            if (stopTurnTorque)
            {
                SetStiffness(frontLeftWheelCollider, flFwd0.stiffness, flSide0.stiffness * 10f);
                carRigidbody.linearVelocity -=  horizontalInput* transform.right * 2f;
            }

            if (horizontalInput > 0)
            {
                carRigidbody.AddTorque(Vector3.up * horizontalInput * driftYawPower, ForceMode.Acceleration);
            }
            else if (horizontalInput < 0)
            {
                carRigidbody.AddTorque(Vector3.up * horizontalInput * driftYawPower, ForceMode.Acceleration);
            }

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
            RestoreFriction();
            // keep stopAngleChange as set by pose logic
        }

        // === NEW: clamp yaw while drifting ===
        if (isDrifting)
        {
            float currentYaw = transform.eulerAngles.y;
            float deltaFromStart = Mathf.DeltaAngle(driftStartYaw, currentYaw);
            float dynMaxDriftYaw = maxDriftYaw;
            if (horizontalInput > 0 && Input.GetKey(KeyCode.A))
            {
                //drifting right while holding left
                Debug.Log("drifting right while holding left");
                stopTurnTorque = true;
                if (stuckYawIfOppositeHeld == 0f)
                {
                    stuckYawIfOppositeHeld = currentYaw;
                }
                else
                {
                    dynMaxDriftYaw = currentYaw;
                }

            }
            else if (horizontalInput < 0 && Input.GetKey(KeyCode.D))
            {
                //drifting left while holding right
                Debug.Log("drifting left while holding right");
                stopTurnTorque = true;
                if (stuckYawIfOppositeHeld == 0f)
                {
                    stuckYawIfOppositeHeld = currentYaw;
                }
                else
                {
                    dynMaxDriftYaw = currentYaw;
                }

            }
            else
            {
                stopTurnTorque = false;
            }
            float clampedDelta = Mathf.Clamp(deltaFromStart, -dynMaxDriftYaw, dynMaxDriftYaw);
            float targetYaw = driftStartYaw + clampedDelta;

            Vector3 e = transform.eulerAngles;
            e.y = targetYaw;



        }
        else
        {
            stuckYawIfOppositeHeld = 0f;
            stopTurnTorque = false;
        }
        // =====================================

        if (Mathf.Abs(horizontalInput) >= 0.1f)
        {
            float tilt = bodyVisual.localEulerAngles.z;
            tilt = Mathf.LerpAngle(tilt, steerAngle * 0.3f * (isDrifting ? bodyVisualMultiplierDrift * 4 : bodyVisualMultiplier), Time.deltaTime * 10f);
            bodyVisual.localEulerAngles = new Vector3(initalBodyRotation.x, initalBodyRotation.y, tilt);
        }
        else
        {
            float tilt = bodyVisual.localEulerAngles.z;
            tilt = Mathf.LerpAngle(tilt, initalBodyRotation.z, Time.deltaTime * 10f);
            bodyVisual.localEulerAngles = new Vector3(initalBodyRotation.x, initalBodyRotation.y, tilt);
        }
        if (carRigidbody.linearVelocity.magnitude > maxSpeed)
        {
            float targetMag = Mathf.Lerp(carRigidbody.linearVelocity.magnitude, maxSpeed, Time.deltaTime * 10f);
            carRigidbody.linearVelocity = carRigidbody.linearVelocity.normalized * targetMag;
            if (carRigidbody.linearVelocity.magnitude > maxSpeedHardCap)
            {
                carRigidbody.linearVelocity = carRigidbody.linearVelocity.normalized * maxSpeedHardCap;
            }
        }
        if (isGrounded)
        {
            carRigidbody.centerOfMass = normalCOM;
        }
        else
        {
            carRigidbody.centerOfMass = airCOM;
        }
    }
    public float wheelCastDist = 0.35f;
    public void CheckGround()
    {
        RaycastHit hit;
        if (Physics.Raycast(transform.position, Vector3.down, out hit, 1f))
        {
            isGrounded = true;
        }
        else
        {
            isGrounded = false;
        }
        Debug.DrawRay(transform.position, Vector3.down * 1f, Color.red);

        

        RaycastHit leftWheelHit;
        if (Physics.Raycast(rearLeftWheelTransform.position, Vector3.down, out leftWheelHit, wheelCastDist))
        {
            leftBackWheelIsGrounded = true;
            leftBackWheelTrail.emitting = true;
//            Debug.Log(leftWheelHit.collider.name);
        }
        else
        {
            leftBackWheelIsGrounded = false;
            leftBackWheelTrail.emitting = false;
        }
        Debug.DrawRay(rearLeftWheelTransform.position, Vector3.down * wheelCastDist, Color.blue);

        RaycastHit rightWheelHit;
        if (Physics.Raycast(rearRightWheelTransform.position, Vector3.down, out rightWheelHit, wheelCastDist))
        {
            rightBackWheelIsGrounded = true;
            rightBackWheelTrail.emitting = true;
        }
        else
        {
            rightBackWheelIsGrounded = false;
            rightBackWheelTrail.emitting = false;
        }
        Debug.DrawRay(rearRightWheelTransform.position, Vector3.down * wheelCastDist, Color.blue);
    }

    void Update()
    {
        
        CheckGround();
        var shapeModule = windEffect.shape;
        shapeModule.radius = Mathf.Lerp(windMaxRadius, windMinRadius, velocity / maxSpeed);
        if (velocity <= 3)
        {
            shapeModule.radius = 100;
            windEffect.Stop();
        }
        else
        {
            windEffect.Play();
        }
        speedText.text = ((int)(CarController.velocity * 3.6)).ToString();
        speedText.color = Color.Lerp(Color.white, Color.yellow, velocity / maxSpeed);
        speedText.fontSize = Mathf.Lerp(32, 37, velocity / maxSpeed);
        if (Input.GetKeyDown(KeyCode.Space) && isGrounded)
        {
            carRigidbody.linearVelocity += new Vector3(0f, 20f, 0f);
            Boost(20);
        }
        if (Input.GetKeyDown(KeyCode.E) && isGrounded)
        {
            Boost(30);
        }
        if (Input.GetKey(KeyCode.E))
        {
            isBreaking = true;
        }
        else
        {
            isBreaking = false;
        }

        // =================== NEW: Drift staging logic ===================
        bool driftHeld = Input.GetKey(driftKey);
        bool canDrift = isGrounded && horizontalInput != 0 && verticalInput != 0;

        // initial press -> start "pose"
        if (driftHeld && !driftKeyWasHeld && canDrift && !isDrriftActiveAny())
        {
            isDriftPosing = true;
            isDrifting = false;
            didActualDrift = false;
            driftPoseTimer = 0f;
            driftTotalTimer = 0f;
            carRigidbody.linearVelocity += new Vector3(0f, 16f, 0f);

            initYStartDrift = transform.eulerAngles.y; // store yaw
            driftStartYaw = initYStartDrift;

            stopAngleChange = true; // lock into drift pose
            driftTotalTime = 0f;
        }

        // while posing
        if (isDriftPosing)
        {
            driftPoseTimer += Time.deltaTime;
            driftTotalTimer += Time.deltaTime;
            driftTotalTime += Time.deltaTime;
            // after 0.5s of posing -> actual drifting
            if (driftPoseTimer >= timeToStartDriftEffect && canDrift && driftHeld)
            {
                isDriftPosing = false;
                isDrifting = true;
                didActualDrift = true;
                lastDriftDir = horizontalInput;
                stopAngleChange = false; // free rotation for the drift physics

                driftElapsedTime = 0f;   // reset effect timer for VFX gating
                 // your original "enter drift" pop
            }
        }

        // while actually drifting
        if (isDrifting)
        {
            driftElapsedTime += Time.deltaTime;   // keeps your VFX timing intact (0.5s after actual drift)
            driftTotalTimer += Time.deltaTime;
            driftTotalTime += Time.deltaTime;
            bool timeUp = driftTotalTimer >= maxDriftTime;
            bool released = !driftHeld;

            if (timeUp || released || !canDrift)
            {
                // exiting real drift -> apply boost
                if (didActualDrift)
                {
                    carRigidbody.linearVelocity -= transform.forward * driftEndBoost * 1.5f;

                    carRigidbody.linearVelocity += transform.right * (lastDriftDir * driftEndBoost * 0.1f * driftTotalTime);
                }
                boostEffect.Play();
                vfxBoostEffect.Play();
                isDrifting = false;
                isDriftPosing = false;
                didActualDrift = false;
                stopAngleChange = false;
                driftPoseTimer = 0f;
                driftTotalTimer = 0f;
                driftTotalTime = 0f;
            }
        }

        // if key released early during pose (never reached actual drift) -> just cancel pose
        if (!driftHeld && isDriftPosing)
        {
            isDriftPosing = false;
            stopAngleChange = false;
            driftPoseTimer = 0f;
            driftTotalTimer = 0f;
            didActualDrift = false;
        }

        // no drift state -> reset timers used by VFX gating
        if (!isDriftPosing && !isDrifting)
        {
            driftElapsedTime = 0f;
        }

        driftKeyWasHeld = driftHeld;
        // ================================================================

        if (!isGrounded)
        {
            Physics.gravity = airGravity;
        }
        else
        {
            Physics.gravity = normalGravity;
        }
        if (Input.GetKey(KeyCode.R))
        {
            SceneManager.LoadScene(SceneManager.GetActiveScene().name);
        }
        if (isDrifting && driftElapsedTime > timeToStartDriftEffect)
        {
            if (horizontalInput > 0)
            {
                leftDriftEffect.Play();
                rightDriftEffect.Stop();
            }
            else
            {
                leftDriftEffect.Stop();
                rightDriftEffect.Play();
            }
        }
        else
        {
            leftDriftEffect.Stop();
            rightDriftEffect.Stop();
        }
    }

    // helper to know if any drift state is currently active
    private bool isDrriftActiveAny()
    {
        return isDriftPosing || isDrifting;
    }
    public void Boost(float force)
    {
        boostEffect.Play();
        vfxBoostEffect.Play();
        carRigidbody.linearVelocity -= transform.forward * force;
    }

    void TuneFriction(WheelCollider wc, float fStiff=3f, float sStiff=3f)
    {
        var f = wc.forwardFriction;
        f.extremumSlip   = 0.35f;
        f.extremumValue  = 1.20f;
        f.asymptoteSlip  = 0.80f;
        f.asymptoteValue = 1.00f;
        f.stiffness      = fStiff;
        wc.forwardFriction = f;

        var s = wc.sidewaysFriction;
        s.extremumSlip   = 0.18f;
        s.extremumValue  = 1.00f;
        s.asymptoteSlip  = 0.45f;
        s.asymptoteValue = 0.80f;
        s.stiffness      = sStiff;
        wc.sidewaysFriction = s;
    }
    public float frontRayCastDist = 3f;
    private void GetInput()
    {
        horizontalInput = Input.GetAxis("Horizontal");
        
        verticalInput = -Input.GetAxis("Vertical");
        int mask = ~excludePushLayers;
        if (Physics.Raycast(transform.position + transform.up.normalized * 0.5f, -transform.forward, out RaycastHit colHit, frontRayCastDist, mask) && colHit.collider.gameObject.layer != excludePushLayers)
        {
            //Against a wall, so help player back up
            Debug.Log(colHit.collider.gameObject.name);
            Debug.Log(colHit.collider.gameObject.layer);
            Debug.Log("back up");
            verticalInput = 1;
            if (horizontalInput == 0)
            {
                horizontalInput = 1;
            }
            carRigidbody.linearVelocity += transform.forward * (Mathf.Lerp(4f, 0.4f, velocity /maxSpeed));
            Vector3 flattened = Vector3.ProjectOnPlane(colHit.normal, Vector3.up); //flatten the collision vector(not really that important)
            float signedAngle = Vector3.SignedAngle(flattened, -transform.forward, Vector3.up); //get the difference of angle between car forward and the collision, so we know what angle the car struck wall
            carRigidbody.angularVelocity -= Math.Sign(signedAngle) * Vector3.up * 9f;

        }
        Debug.DrawRay(transform.position, -transform.forward * frontRayCastDist, Color.red);
        lastVert = verticalInput;
    }

    private float yawBrake = 10f;

    private void HandleSteering()
    {
        // steer harder during real drift; pose uses stopAngleChange in FixedUpdate
        if (stopTurnTorque)
        {
            frontLeftWheelCollider.steerAngle = 0f;
            frontRightWheelCollider.steerAngle = 0f;
            return;
        }
        steerAngle = maxSteeringAngle * horizontalInput * (isDrifting ? driftSteerMultiplier : 1f);

        frontLeftWheelCollider.steerAngle  = steerAngle * 0.5f;
        frontRightWheelCollider.steerAngle = steerAngle * 0.5f;

        if (Mathf.Abs(horizontalInput) < 0.01f)
        {
            Vector3 w = carRigidbody.angularVelocity;
            float yaw = Vector3.Dot(w, Vector3.up);
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
        if (isDrifting)
        {
            frontLeftWheelCollider.motorTorque  = verticalInput * motorForce * driftTorqueMultiplier;
            frontRightWheelCollider.motorTorque = verticalInput * motorForce * driftTorqueMultiplier;

            frontLeftWheelCollider.brakeTorque  = 0f;
            frontRightWheelCollider.brakeTorque = 0f;
            rearLeftWheelCollider.brakeTorque   = handbrakeTorque;
            rearRightWheelCollider.brakeTorque  = handbrakeTorque;
            return;
        }

        frontLeftWheelCollider.motorTorque = verticalInput * motorForce;
        frontRightWheelCollider.motorTorque = verticalInput * motorForce;

       // HandleMotor()
bool playerBraking = isBreaking || (verticalInput > 0.1f); // pressing reverse counts as brake
float appliedBrake = playerBraking ? 2000f : 0f;

frontLeftWheelCollider.brakeTorque  = appliedBrake;
frontRightWheelCollider.brakeTorque = appliedBrake;
rearLeftWheelCollider.brakeTorque   = appliedBrake;
rearRightWheelCollider.brakeTorque  = appliedBrake;

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
}

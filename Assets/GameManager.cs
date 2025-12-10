using System;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.UI;

public class GameManager : MonoBehaviour
{
    public static bool initalCutscenePlayed;
    public static bool gameStarted;
    public CarController carController;

    public float startGameAfter = 3;

    public Image panelColor;
    public TextMeshProUGUI countdownText;
    public CameraFollow camFollow;
    public AnimationClip initalCutsceneAnim;
    public Animator camAnimator;
    public LapProgress lapProgress;
    public AudioSource countdownSound;
    public List<List<GameObject>> objectsForLaps;
    void Start()
    {
        gameStarted = false;
        RenderSettings.fogDensity = 0.001f;
        if (!initalCutscenePlayed)
        {
            carController.gameObject.SetActive(false);
            camAnimator.enabled = true;
            camFollow.enabled = false;
            StartCoroutine(InitalCutscene());
        }else
        {
            // restart the lap, already played cutscene before
            countdownSound.Play();
            RenderSettings.fogDensity = 0.003f;
        }
    }

    // Update is called once per frame
    void Update()
    {
        if (!initalCutscenePlayed)
        {
            return;
        }
        if (!gameStarted)
        {
            startGameAfter -= Time.deltaTime;
            Color targetColor = panelColor.color;
            targetColor.a = Mathf.Lerp(0.1f, 0.02f, 1 / startGameAfter);
            panelColor.color = targetColor;

            countdownText.text = Math.Ceiling((double)startGameAfter).ToString();
        }

        if (startGameAfter <= 0 && !gameStarted)
        {
            gameStarted = true;
            carController.enabled = true;
            panelColor.enabled = false;
            countdownText.text = "";
            carController.StartCar();
            camFollow.gameStarted = true;
        }
    }
    public IEnumerator InitalCutscene()
    {
        camAnimator.Play(initalCutsceneAnim.name);
       yield return new WaitForSeconds(initalCutsceneAnim.length);
       //yield return new WaitForSeconds(1);
        initalCutscenePlayed = true;
        carController.gameObject.SetActive(true);
        camAnimator.enabled = false;
        camFollow.enabled = true;
        countdownSound.Play();
        RenderSettings.fogDensity = 0.003f;
    }
}

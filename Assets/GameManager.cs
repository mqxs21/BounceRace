using System;
using System.Collections;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class GameManager : MonoBehaviour
{
    public static bool gameStarted;
    public CarController carController;

    public float startGameAfter = 3;

    public Image panelColor;
    public TextMeshProUGUI textMeshProUGUI;
    public CameraFollow camFollow;
    void Start()
    {
        gameStarted = false;
        
    }

    // Update is called once per frame
    void Update()
    {
        if (!gameStarted)
        {
            startGameAfter -= Time.deltaTime;
            Color targetColor = panelColor.color;
            targetColor.a = Mathf.Lerp(0.7f, 0.02f, 1/startGameAfter);
            panelColor.color = targetColor;

            textMeshProUGUI.text = Math.Ceiling((double)startGameAfter).ToString();
        }

        if (startGameAfter <= 0 && !gameStarted)
        {
            gameStarted = true;
            carController.enabled = true;
            panelColor.enabled = false;
            textMeshProUGUI.text = "";
            carController.StartCar();
            camFollow.gameStarted = true;
        }
    }
}

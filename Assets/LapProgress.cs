using System.Collections;
using TMPro;
using UnityEngine;

public class LapProgress : MonoBehaviour
{
    public int currentLap = 0;
    public int currentCheckpoint = 0;
    public int totalCheckpoints = 0;
    public int totalLaps = 3;

    public float lapTimeElapsed = 0f;
    public float bestLapTime = 0f;
    public TextMeshProUGUI lapTimeText;

    public TextMeshProUGUI lapText;

    public static bool gameIsDone = false;
    public static bool gameIsFinalDone = false;
    public CarController carController;
    public AudioSource finishTrackSound;
    public AudioSource whooshSound;

    public Animator gameFinishAnimator;
    public AnimationClip gameFinishClip;
    
    void Start()
    {
        gameIsDone = false;
    }

    // Update is called once per frame
    void Update()
    {
        if (gameIsDone)
        {
            return;
        }
        UpdateUI();
    }
    public void CheckPointReached(int checkpointIndex, bool isFinish = false)
    {
        if (checkpointIndex == currentCheckpoint + 1)
        {
            currentCheckpoint++;
        }
        else
        {
            Debug.Log("Wrong way");
        }
        LapCompleted(isFinish);
    }

    public void LapCompleted(bool isFinish)
    {
        if (currentCheckpoint == totalCheckpoints && isFinish)
        {
            if (currentLap == totalLaps)
            {
                FinishGame();
                return;
            }
            currentLap++;
            currentCheckpoint = 1;
            if (lapTimeElapsed < bestLapTime)
            {
                bestLapTime = lapTimeElapsed;
            }
            lapTimeElapsed = 0f;
        }
    }
    void UpdateUI()
    {
        if(GameManager.gameStarted) lapTimeElapsed += Time.deltaTime;
        string formattedTime = ((float)(lapTimeElapsed)).ToString("F2");
        lapTimeText.text = formattedTime.ToString();
        lapText.text = ((int)(currentLap)).ToString() + "/" + ((int)(totalLaps)).ToString();
    }
    void FinishGame()
    {
        gameIsDone = true;

        UpdateUI();

        StartCoroutine(GameFinishedCoroutine());

    }
    IEnumerator GameFinishedCoroutine()
    {
        finishTrackSound.Play();
        Time.timeScale = 0.2f;
        yield return new WaitForSecondsRealtime(1f);
        gameIsFinalDone = true;
        whooshSound.Play();
        gameFinishAnimator.Play(gameFinishClip.name);
        Time.timeScale = 1f;
    }
}

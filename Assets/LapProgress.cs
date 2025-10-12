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
    public CarController carController;
    
    void Start()
    {
        gameIsDone = false;
    }

    // Update is called once per frame
    void Update()
    {
        if(GameManager.gameStarted) lapTimeElapsed += Time.deltaTime;
        string formattedTime = ((float)(lapTimeElapsed)).ToString("F2");
        lapTimeText.text = formattedTime.ToString();
        lapText.text = ((int)(currentLap)).ToString() + "/" + ((int)(totalLaps)).ToString();
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
    void FinishGame()
    {
        gameIsDone = true;
        carController.canMove = false;
    }
}

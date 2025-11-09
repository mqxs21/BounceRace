using UnityEngine;
using UnityEngine.SceneManagement;

public class SceneSetter : MonoBehaviour
{
    void Awake()
    {
        if (SceneManager.GetActiveScene().name == "StartMenu")
        {
            GameManager.initalCutscenePlayed = false;
        }
    }
    public void GoToTrack1()
    {
        SceneManager.LoadScene("track1");
    }
    public void RestartScene()
    {
        SceneManager.LoadScene(SceneManager.GetActiveScene().name);
    }
    public void GoToStartMenu()
    {
        SceneManager.LoadScene("StartMenu");
    }
}
